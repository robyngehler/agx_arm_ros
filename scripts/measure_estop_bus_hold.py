#!/usr/bin/env python3
"""Record what the CAN link does across an external-watchdog emergency stop (L3).

The question this answers: when the watchdog takes the bus on an e-stop and
gives it back on release, does the driver wait the hold out or does it run a
recovery against a bus that was never broken?

That decision is `_bus_hold_defers_recovery()` in agx_arm_ctrl_single_node.py,
and it is made from two kernel counters — `/sys/class/net/<iface>/statistics/
{rx,tx}_packets`. This samples the same two counters, so the report replays the
gate offline and says what it would have seen and why. The controller error
state (ERROR-ACTIVE/WARNING/PASSIVE and the transmit error counter) is neither
on the wire nor in sysfs, so it comes from `ip -d link show`, polled fast only
while something is happening.

No operator input is needed while it runs. The e-stop is the moment RX goes
silent and the release is the moment RX comes back, so the report derives the
timeline from the counters and anchors it against the driver log.

Read-only: no ROS node, no SDK session, no frame is transmitted. The pcap is
optional and needs sudo for tcpdump.

Usage:
    # start it, do the run, come back
    python3 scripts/measure_estop_bus_hold.py record --iface can_nero_left --duration 240

    # afterwards
    python3 scripts/measure_estop_bus_hold.py report --log /tmp/estop_run.log
"""

from __future__ import annotations

import argparse
import json
import os
import re
import signal
import subprocess
import sys
import time
from datetime import datetime
from pathlib import Path

DEFAULT_ROOT = Path("docs/sprint_refactor/reference/measurements/estop_bus_hold")

# The driver's own gate constants (agx_arm_ctrl_single_node.py). Kept here so the
# report replays the same thresholds; a change there must change these too.
BUS_HOLD_MIN_SILENCE_S = 0.25
BUS_HOLD_PATIENCE_S = 60.0

STATE_RE = re.compile(
    r"can .*?state (?P<state>[A-Z-]+) \(berr-counter tx (?P<tec>\d+) rx (?P<rec>\d+)\)"
)
STATE_NO_BERR_RE = re.compile(r"can .*?state (?P<state>[A-Z-]+)")
COUNTER_HEADER_RE = re.compile(
    r"re-started\s+bus-errors\s+arbit-lost\s+error-warn\s+error-pass\s+bus-off"
)


def sudo_prefix() -> list[str]:
    """Empty when already root. The password is disabled in the hardware env."""
    return [] if os.geteuid() == 0 else ["sudo", "-n"]


# --------------------------------------------------------------------------
# record
# --------------------------------------------------------------------------


class LinkSampler:
    """rx/tx packet counters at `fast_hz`, controller state adaptively.

    `ip` is a subprocess per poll and this Jetson has little CPU headroom, so it
    runs at `idle_hz` while the link is quiet and healthy and at `busy_hz` while
    RX is silent or the controller is not ERROR-ACTIVE — which is the whole
    window of interest and costs nothing the rest of the time.
    """

    def __init__(self, iface: str, fast_hz: float, idle_hz: float, busy_hz: float):
        self.iface = iface
        self.period = 1.0 / fast_hz
        self.idle_period = 1.0 / idle_hz
        self.busy_period = 1.0 / busy_hz
        base = Path("/sys/class/net") / iface / "statistics"
        self._rx = open(base / "rx_packets")
        self._tx = open(base / "tx_packets")
        self._last_ip = 0.0
        self._state = ""
        self._tec = self._rec = -1
        self._tallies = (-1,) * 6
        self._last_rx_value = None
        self._last_rx_advance = time.monotonic()

    def _counter(self, handle) -> int:
        handle.seek(0)
        return int(handle.read().strip())

    def _poll_ip(self) -> None:
        try:
            out = subprocess.run(
                ["ip", "-d", "-s", "link", "show", self.iface],
                capture_output=True, text=True, timeout=2,
            ).stdout
        except Exception:
            return
        match = STATE_RE.search(out)
        if match:
            self._state = match.group("state")
            self._tec = int(match.group("tec"))
            self._rec = int(match.group("rec"))
        else:
            fallback = STATE_NO_BERR_RE.search(out)
            if fallback:
                self._state = fallback.group("state")
        lines = out.splitlines()
        for index, line in enumerate(lines):
            if COUNTER_HEADER_RE.search(line) and index + 1 < len(lines):
                fields = lines[index + 1].split()[:6]
                if len(fields) == 6 and all(f.isdigit() for f in fields):
                    self._tallies = tuple(int(f) for f in fields)
                break

    def sample(self) -> dict:
        now_mono = time.monotonic()
        rx = self._counter(self._rx)
        tx = self._counter(self._tx)
        if self._last_rx_value is None or rx != self._last_rx_value:
            self._last_rx_advance = now_mono
        self._last_rx_value = rx
        silent_for = now_mono - self._last_rx_advance

        quiet_and_healthy = silent_for < 0.05 and self._state == "ERROR-ACTIVE"
        due = self.idle_period if quiet_and_healthy else self.busy_period
        if now_mono - self._last_ip >= due:
            self._last_ip = now_mono
            self._poll_ip()

        restarted, bus_errors, arbit_lost, warn, passive, bus_off = self._tallies
        return {
            "t": time.time(),
            "rx_packets": rx,
            "tx_packets": tx,
            "rx_silent_s": round(silent_for, 4),
            "state": self._state,
            "tec": self._tec,
            "rec": self._rec,
            "restarted": restarted,
            "bus_errors": bus_errors,
            "arbit_lost": arbit_lost,
            "error_warn": warn,
            "error_pass": passive,
            "bus_off": bus_off,
        }


FIELDS = [
    "t", "rx_packets", "tx_packets", "rx_silent_s", "state", "tec", "rec",
    "restarted", "bus_errors", "arbit_lost", "error_warn", "error_pass", "bus_off",
]


def start_pcap(iface: str, path: Path):
    """tcpdump on the CAN interface, or None when it cannot start.

    Non-fatal: the counter trace is the primary record and stands on its own.
    """
    cmd = sudo_prefix() + ["tcpdump", "-i", iface, "-U", "-w", str(path)]
    try:
        proc = subprocess.Popen(cmd, stdout=subprocess.DEVNULL, stderr=subprocess.PIPE)
    except FileNotFoundError:
        print("  no tcpdump; continuing without a pcap")
        return None
    time.sleep(0.7)
    if proc.poll() is not None:
        err = (proc.stderr.read() or b"").decode().strip()
        print(f"  tcpdump did not start ({err}); continuing without a pcap")
        return None
    print(f"  pcap -> {path}")
    return proc


def stop_pcap(proc, path: Path) -> None:
    if proc is None:
        return
    # By write path, not the Popen handle: that handle may be sudo's.
    subprocess.run(
        sudo_prefix() + ["pkill", "-INT", "-f", f"tcpdump.*-w {path}"],
        capture_output=True,
    )
    try:
        proc.wait(timeout=5)
    except subprocess.TimeoutExpired:
        proc.kill()


def cmd_record(args: argparse.Namespace) -> int:
    root = Path(args.root)
    run_dir = root / datetime.now().strftime("%Y%m%d_%H%M%S")
    run_dir.mkdir(parents=True, exist_ok=True)
    latest = root / "latest"
    if latest.is_symlink() or latest.exists():
        latest.unlink()
    latest.symlink_to(run_dir.name)

    (run_dir / "meta.json").write_text(json.dumps({
        "iface": args.iface,
        "started_epoch": time.time(),
        "started_iso": datetime.now().isoformat(timespec="seconds"),
        "fast_hz": args.fast_hz,
        "bus_hold_min_silence_s": BUS_HOLD_MIN_SILENCE_S,
        "note": args.note,
    }, indent=2) + "\n")

    sampler = LinkSampler(args.iface, args.fast_hz, args.idle_ip_hz, args.busy_ip_hz)
    stopping = {"now": False}
    signal.signal(signal.SIGINT, lambda *_: stopping.__setitem__("now", True))
    signal.signal(signal.SIGTERM, lambda *_: stopping.__setitem__("now", True))

    pcap = start_pcap(args.iface, run_dir / f"{args.iface}.pcap") if args.pcap else None

    limit = f"{args.duration:.0f}s or Ctrl-C" if args.duration else "Ctrl-C"
    print(f"recording {args.iface} -> {run_dir}")
    print(f"  runs until {limit}; no input needed while it runs")

    path = run_dir / "link.csv"
    deadline = time.monotonic() + args.duration if args.duration else None
    written = 0
    # Announced live so a run can be sanity-checked at the terminal afterwards
    # without waiting for the report.
    last_rx_live = None
    with path.open("w", buffering=1) as out:
        out.write(",".join(FIELDS) + "\n")
        next_tick = time.monotonic()
        while not stopping["now"]:
            row = sampler.sample()
            out.write(",".join(str(row[f]) for f in FIELDS) + "\n")
            written += 1
            silent = row["rx_silent_s"] >= BUS_HOLD_MIN_SILENCE_S
            if last_rx_live is None or silent != last_rx_live:
                last_rx_live = silent
                mark = "RX SILENT" if silent else "RX live"
                print(f"  {datetime.now().strftime('%H:%M:%S.%f')[:-3]}  {mark}"
                      f"  state={row['state'] or '?'} tec={row['tec']}")
            if deadline and time.monotonic() >= deadline:
                break
            next_tick += sampler.period
            delay = next_tick - time.monotonic()
            if delay > 0:
                time.sleep(delay)
            else:
                next_tick = time.monotonic()

    stop_pcap(pcap, run_dir / f"{args.iface}.pcap")
    print(f"\n{written} samples -> {path}")
    print(f"report with:\n  python3 {sys.argv[0]} report --run {run_dir} --log <launch log>")
    return 0


# --------------------------------------------------------------------------
# report
# --------------------------------------------------------------------------


def resolve_run(args: argparse.Namespace) -> Path:
    if getattr(args, "run", None):
        return Path(args.run)
    latest = Path(args.root) / "latest"
    if not latest.exists():
        sys.exit(f"no run given and no {latest}")
    return latest.resolve()


def load_rows(path: Path) -> list[dict]:
    rows = []
    with path.open() as handle:
        header = handle.readline().strip().split(",")
        for line in handle:
            values = line.strip().split(",")
            if len(values) != len(header):
                continue
            row = dict(zip(header, values))
            row["t"] = float(row["t"])
            row["rx_silent_s"] = float(row["rx_silent_s"])
            for key in ("rx_packets", "tx_packets", "tec", "rec", "restarted",
                        "bus_errors", "arbit_lost", "error_warn", "error_pass",
                        "bus_off"):
                row[key] = int(row[key])
            rows.append(row)
    return rows


def silence_episodes(rows: list[dict], counter: str, min_s: float) -> list[dict]:
    """Runs where `counter` did not advance for at least `min_s`.

    Derived rather than marked: on this rig the operator has both hands on the
    robot, and the events worth timestamping are exactly the ones the counters
    already show.
    """
    episodes = []
    last_value = None
    last_advance_index = 0
    open_from = None
    for index, row in enumerate(rows):
        if last_value is None or row[counter] != last_value:
            if open_from is not None:
                start = rows[last_advance_index]["t"]
                episodes.append({
                    "start": start,
                    "end": row["t"],
                    "duration": row["t"] - start,
                    "start_index": last_advance_index,
                    "end_index": index,
                })
                open_from = None
            last_value = row[counter]
            last_advance_index = index
        elif row["t"] - rows[last_advance_index]["t"] >= min_s and open_from is None:
            open_from = last_advance_index
    if open_from is not None:
        start = rows[last_advance_index]["t"]
        episodes.append({
            "start": start,
            "end": None,
            "duration": rows[-1]["t"] - start,
            "start_index": last_advance_index,
            "end_index": len(rows) - 1,
        })
    return episodes


def gate_verdict(rows: list[dict], episode: dict) -> dict:
    """Replay `_bus_hold_defers_recovery`'s entry test for one RX silence.

    Entry needs TX to have advanced since the *previous sample* at the moment RX
    silence crosses the threshold. Reported alongside how long TX actually kept
    advancing after RX stopped, which is the margin the condition runs on.
    """
    start_index = episode["start_index"]
    crossing_index = None
    for index in range(start_index, episode["end_index"] + 1):
        if rows[index]["t"] - episode["start"] >= BUS_HOLD_MIN_SILENCE_S:
            crossing_index = index
            break
    if crossing_index is None:
        return {"crossed": False}
    tx_at_crossing = rows[crossing_index]["tx_packets"]
    tx_accepted = (
        crossing_index > 0
        and tx_at_crossing > rows[crossing_index - 1]["tx_packets"]
    )
    last_tx_advance = episode["start"]
    tx_before = rows[start_index]["tx_packets"]
    for index in range(start_index, episode["end_index"] + 1):
        row = rows[index]
        if row["tx_packets"] > tx_before:
            tx_before = row["tx_packets"]
            last_tx_advance = row["t"]
        elif row["t"] - last_tx_advance >= BUS_HOLD_MIN_SILENCE_S:
            # TX has gone quiet for as long as the gate's own threshold; anything
            # later is the stream coming back, not the stream still running.
            break
    quiet_index = max(start_index, episode["end_index"] - 1)
    return {
        "crossed": True,
        "crossed_at": rows[crossing_index]["t"],
        "tx_accepted": tx_accepted,
        "tx_advanced_for_s": last_tx_advance - episode["start"],
        "tx_during_silence": (
            rows[quiet_index]["tx_packets"] - rows[start_index]["tx_packets"]
        ),
        "state": rows[crossing_index]["state"],
        "tec": rows[crossing_index]["tec"],
    }


DECISIVE_LOG_PATTERNS = [
    ("HOLD ENTERED", "CAN RX silent for"),
    ("HOLD RELEASED", "CAN RX resumed after"),
    ("hold gave up", "past the"),
    ("RECOVERY FIRED", "CAN bus stall detected"),
    ("no hold pose", "pre-recovery firmware hold UNAVAILABLE"),
    ("hold established", "pre-recovery firmware hold established"),
    ("FAULT LOCKOUT", "fault lockout"),
    ("recovery retry", "did not restore feedback"),
    ("recovery done", "recovery finished after"),
    ("starvation", "treating as local starvation"),
    ("acq rate", "acquisition loop"),
    ("read thread died", "Bad file descriptor"),
    ("control re-armed", "control is now enabled"),
    ("claim", "claimed by"),
    ("traj accepted", "Accepted FollowJointTrajectory"),
    ("traj aborted", "ABORTED"),
    ("traj ok", "SUCCEEDED"),
]

LOG_TS_RE = re.compile(r"\[(\d{10}\.\d+)\]")


def scan_log(path: Path) -> list[tuple[float, str, str]]:
    hits = []
    with path.open(errors="replace") as handle:
        for line in handle:
            for label, needle in DECISIVE_LOG_PATTERNS:
                if needle in line:
                    match = LOG_TS_RE.search(line)
                    hits.append((float(match.group(1)) if match else 0.0,
                                 label, line.strip()))
                    break
    return hits


def cmd_report(args: argparse.Namespace) -> int:
    run_dir = resolve_run(args)
    rows = load_rows(run_dir / "link.csv")
    if not rows:
        sys.exit(f"no samples in {run_dir}/link.csv")
    t0 = rows[0]["t"]

    def rel(stamp) -> str:
        return "      -" if stamp is None else f"{stamp - t0:+7.2f}s"

    print(f"run: {run_dir}")
    print(f"samples: {len(rows)}  span: {rows[-1]['t'] - t0:.1f}s")
    print("t=0 is the first sample; the e-stop is the RX-silent edge below.\n")

    rx_eps = silence_episodes(rows, "rx_packets", BUS_HOLD_MIN_SILENCE_S)
    tx_eps = silence_episodes(rows, "tx_packets", BUS_HOLD_MIN_SILENCE_S)

    events: list[tuple[float, str, str]] = []
    for episode in rx_eps:
        events.append((episode["start"], "BUS",
                       f"RX silent  <- watchdog took the bus (e-stop)"))
        if episode["end"]:
            events.append((episode["end"], "BUS",
                           f"RX back after {episode['duration']:.2f}s  <- bus given back"))
    for episode in tx_eps:
        events.append((episode["start"], "CMD", "TX stopped  <- command stream off"))
        if episode["end"]:
            events.append((episode["end"], "CMD",
                           f"TX resumed after {episode['duration']:.2f}s"))
    previous_state = None
    previous_tec = None
    for row in rows:
        if row["state"] and row["state"] != previous_state:
            if previous_state is not None:
                events.append((row["t"], "CAN", f"state -> {row['state']}"))
            previous_state = row["state"]
        if row["tec"] >= 0:
            if previous_tec is not None:
                if previous_tec == 0 and row["tec"] > 0:
                    events.append((row["t"], "CAN", f"TEC rising (tec={row['tec']})"))
                elif previous_tec > 0 and row["tec"] == 0:
                    events.append((row["t"], "CAN", "TEC back to 0"))
            previous_tec = row["tec"]

    log_hits = scan_log(Path(args.log)) if args.log else []
    for stamp, label, line in log_hits:
        if stamp:
            events.append((stamp, "LOG", f"[{label}] {line[:120]}"))

    print("== derived timeline ==")
    for stamp, kind, text in sorted(events, key=lambda e: e[0]):
        print(f"  {rel(stamp)}  {kind:<4} {text}")
    if log_hits and not any(h[0] for h in log_hits):
        print("  (log lines carried no epoch stamp and could not be placed)")

    print("\n== controller state ==")
    if not any(row["state"] for row in rows):
        print("  `ip -d link show` reported no CAN controller state for this "
              "interface;\n  TEC and the error tallies are unavailable.")
    else:
        peak = max(rows, key=lambda r: r["tec"])
        first_nonzero = next((r for r in rows if r["tec"] > 0), None)
        back_to_zero = next(
            (r for r in rows if r["t"] > peak["t"] and r["tec"] == 0), None
        )
        print(f"  first TEC > 0     {rel(first_nonzero['t'] if first_nonzero else None)}")
        print(f"  peak TEC          {rel(peak['t'])}  tec={peak['tec']} "
              f"state={peak['state']}")
        print(f"  TEC back to 0     {rel(back_to_zero['t'] if back_to_zero else None)}"
              + ("" if back_to_zero else "   <- never decayed in this run"))
        last = rows[-1]
        print(f"  tallies (cumulative): error-warn={last['error_warn']} "
              f"error-pass={last['error_pass']} bus-off={last['bus_off']} "
              f"restarted={last['restarted']}")

    print("\n== the driver's hold gate, replayed on these counters ==")
    print(f"  entry needs: link up AND rx silent >= {BUS_HOLD_MIN_SILENCE_S}s "
          f"AND tx_packets advanced since the previous sample")
    if not rx_eps:
        print("  RX never went silent past the threshold — the gate was never asked.")
        print("  If you did trigger the e-stop, the watchdog did not take this bus.")
    for episode in rx_eps:
        verdict = gate_verdict(rows, episode)
        print(f"\n  RX silence {rel(episode['start'])} -> {rel(episode['end'])} "
              f"({episode['duration']:.2f}s)")
        if not verdict["crossed"]:
            print("    never reached the threshold in this episode")
            continue
        print(f"    TX kept advancing for {verdict['tx_advanced_for_s']:.3f}s "
              f"after RX stopped (needs >= {BUS_HOLD_MIN_SILENCE_S}s)")
        print(f"    TX frames during the silence: {verdict['tx_during_silence']}")
        print(f"    state at the crossing: {verdict['state'] or '?'} "
              f"tec={verdict['tec']}")
        if verdict["tx_accepted"]:
            print("    -> WOULD HOLD: recovery deferred, the watchdog is waited out")
        else:
            print("    -> WOULD NOT HOLD: falls through to recovery, which latches")
            print("       the fault lockout that refuses motion after the release")

    print("\n== how to read this ==")
    print("  WOULD HOLD + 'HOLD ENTERED' in the log + TEC back to 0 + a trajectory")
    print("  accepted after the release -> the e-stop/release path works.")
    print("  WOULD NOT HOLD -> the tx_accepted entry condition is the blocker; the")
    print("  watchdog and the bus are fine. Compare 'TX kept advancing' against the")
    print(f"  {BUS_HOLD_MIN_SILENCE_S}s threshold: that difference is the whole margin.")
    return 0


def main() -> int:
    parser = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter
    )
    parser.add_argument("--root", default=str(DEFAULT_ROOT),
                        help="where runs are written (default: %(default)s)")
    sub = parser.add_subparsers(dest="command", required=True)

    rec = sub.add_parser("record", help="sample the link until the duration or Ctrl-C")
    rec.add_argument("--iface", default="can_nero_left")
    rec.add_argument("--duration", type=float, default=0.0,
                     help="seconds to record, 0 = until Ctrl-C (default: %(default)s)")
    rec.add_argument("--fast-hz", type=float, default=100.0,
                     help="counter sampling rate (default: %(default)s)")
    rec.add_argument("--idle-ip-hz", type=float, default=2.0,
                     help="`ip` polling while the link is quiet and healthy")
    rec.add_argument("--busy-ip-hz", type=float, default=20.0,
                     help="`ip` polling while RX is silent or the state is not active")
    rec.add_argument("--no-pcap", dest="pcap", action="store_false",
                     help="skip the tcpdump capture (no sudo needed)")
    rec.add_argument("--note", default="", help="free text stored in meta.json")
    rec.set_defaults(func=cmd_record, pcap=True)

    rep = sub.add_parser("report", help="derive the timeline and replay the hold gate")
    rep.add_argument("--run", help="run directory (default: the latest)")
    rep.add_argument("--log", help="the teed components.launch log")
    rep.set_defaults(func=cmd_report)

    args = parser.parse_args()
    return args.func(args)


if __name__ == "__main__":
    sys.exit(main())
