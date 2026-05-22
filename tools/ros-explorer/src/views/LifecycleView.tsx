import { useEffect, useRef, useState, useMemo } from "react";
import mermaid from "mermaid";
import { useStore } from "../store";
import type { RosNode } from "../types";

mermaid.initialize({
  startOnLoad: false,
  theme: "dark",
  themeVariables: {
    background: "#0f172a",
    primaryColor: "#1e3a5f",
    primaryTextColor: "#e2e8f0",
    lineColor: "#475569",
    edgeLabelBackground: "#1e293b",
    secondaryColor: "#1e293b",
    tertiaryColor: "#0f172a",
  },
});

// ── ROS2 standard lifecycle graph (transitions) ──────────────────────────────
interface Transition { from: string; to: string; trigger: string }

const ROS2_TRANSITIONS: Transition[] = [
  { from: "Unconfigured", to: "Configuring", trigger: "configure()" },
  { from: "Configuring", to: "Inactive", trigger: "on_configure() OK" },
  { from: "Configuring", to: "ErrorProcessing", trigger: "on_configure() FAIL" },
  { from: "Inactive", to: "Activating", trigger: "activate()" },
  { from: "Activating", to: "Active", trigger: "on_activate() OK" },
  { from: "Activating", to: "ErrorProcessing", trigger: "on_activate() FAIL" },
  { from: "Active", to: "Deactivating", trigger: "deactivate()" },
  { from: "Deactivating", to: "Inactive", trigger: "on_deactivate() OK" },
  { from: "Deactivating", to: "ErrorProcessing", trigger: "on_deactivate() FAIL" },
  { from: "Inactive", to: "CleaningUp", trigger: "cleanup()" },
  { from: "CleaningUp", to: "Unconfigured", trigger: "on_cleanup() OK" },
  { from: "CleaningUp", to: "ErrorProcessing", trigger: "on_cleanup() FAIL" },
  { from: "Active", to: "ShuttingDown", trigger: "shutdown()" },
  { from: "Inactive", to: "ShuttingDown", trigger: "shutdown()" },
  { from: "Unconfigured", to: "ShuttingDown", trigger: "shutdown()" },
  { from: "ShuttingDown", to: "Finalized", trigger: "on_shutdown() OK" },
  { from: "ErrorProcessing", to: "Unconfigured", trigger: "on_error() OK" },
  { from: "ErrorProcessing", to: "Finalized", trigger: "on_error() FAIL" },
];

// ── Per-state detail knowledge base ──────────────────────────────────────────
interface StateInfo {
  description: string;
  callbacks: string[];
  topicActivity: "none" | "limited" | "full";
  note?: string;
}

const LIFECYCLE_STATE_INFO: Record<string, StateInfo> = {
  Unconfigured: {
    description: "Node is instantiated but not yet configured. No resources have been allocated.",
    callbacks: ["on_configure()"],
    topicActivity: "none",
    note: "Entry state after construction or after a successful on_error().",
  },
  Configuring: {
    description: "Transient state — on_configure() is executing. Resources are being allocated.",
    callbacks: ["on_configure()"],
    topicActivity: "none",
  },
  Inactive: {
    description: "Node is configured and resources are allocated, but it is not actively processing data.",
    callbacks: ["on_activate()", "on_cleanup()", "on_shutdown()"],
    topicActivity: "limited",
    note: "Publishers and subscribers exist but no data is being processed.",
  },
  Activating: {
    description: "Transient state — on_activate() is executing. Node is being brought online.",
    callbacks: ["on_activate()"],
    topicActivity: "limited",
  },
  Active: {
    description: "Node is fully operational. Publishers, subscribers, timers, and services are all live.",
    callbacks: ["on_deactivate()", "on_shutdown()"],
    topicActivity: "full",
    note: "Normal operating state. All declared topics and services are available.",
  },
  Deactivating: {
    description: "Transient state — on_deactivate() is executing. Node is being paused.",
    callbacks: ["on_deactivate()"],
    topicActivity: "limited",
  },
  CleaningUp: {
    description: "Transient state — on_cleanup() is executing. Resources are being released.",
    callbacks: ["on_cleanup()"],
    topicActivity: "none",
  },
  ShuttingDown: {
    description: "Transient state — on_shutdown() is executing. Node is being destroyed.",
    callbacks: ["on_shutdown()"],
    topicActivity: "none",
  },
  Finalized: {
    description: "Terminal state. Node is fully shut down and ready to be destroyed.",
    callbacks: [],
    topicActivity: "none",
  },
  ErrorProcessing: {
    description: "Error recovery state. on_error() determines if the node can recover to Unconfigured.",
    callbacks: ["on_error()"],
    topicActivity: "none",
    note: "Reached from any transient state when its callback returns FAILURE.",
  },
};

// ── Build transitions for a specific node ────────────────────────────────────
function getNodeTransitions(node: RosNode): Transition[] {
  // Lifecycle nodes always use the standard ROS2 managed-node state machine
  if (node.lifecycleNode) return ROS2_TRANSITIONS;

  // If the scanner captured explicit state info, prefer it
  if (node.lifecycleStates && node.lifecycleStates.length > 0) {
    return node.lifecycleStates.map((s) => ({ from: s.from, to: s.to, trigger: s.trigger }));
  }

  // Regular nodes: derive a minimal state machine from their service names
  const transitions: Transition[] = [
    { from: "[*]", to: "Initializing", trigger: "startup" },
    { from: "Initializing", to: "Ready", trigger: "node started" },
  ];
  if (node.services.some((s) => s.service.includes("enable"))) {
    transitions.push({ from: "Ready", to: "Enabled", trigger: "enable(true)" });
    transitions.push({ from: "Enabled", to: "Ready", trigger: "enable(false)" });
  }
  if (node.services.some((s) => s.service.includes("reset"))) {
    transitions.push({ from: "Enabled", to: "Resetting", trigger: "reset()" });
    transitions.push({ from: "Resetting", to: "Ready", trigger: "reset done" });
  }
  if (node.services.some((s) => s.service.includes("calib") || s.service.includes("trigger"))) {
    transitions.push({ from: "Enabled", to: "Calibrating", trigger: "calibrate()" });
    transitions.push({ from: "Calibrating", to: "Enabled", trigger: "calib done" });
  }
  transitions.push({ from: "Ready", to: "[*]", trigger: "shutdown" });
  if (node.services.some((s) => s.service.includes("enable"))) {
    transitions.push({ from: "Enabled", to: "[*]", trigger: "shutdown" });
  }
  return transitions;
}

// ── State interior diagram: show methods/I/O active inside a specific state ──
function buildStateInteriorDiagram(node: RosNode, stateName: string): string {
  const info = LIFECYCLE_STATE_INFO[stateName];
  // Safe Mermaid ID: alphanumeric + underscores only, no leading digit
  const safe = (s: string) => s.replace(/[^a-zA-Z0-9]/g, "_").replace(/^(\d)/, "N$1");
  const trunc = (s: string, n = 30) => (s.length > n ? s.slice(0, n - 1) + "…" : s);

  const showFull = !info || info.topicActivity === "full";
  const showLimited = info?.topicActivity === "limited";

  const lines: string[] = ["graph LR"];

  // Central state node
  lines.push(`  S["${stateName}"]`);

  // Callbacks this state executes
  const callbacks = info?.callbacks ?? [];
  const cbIds: string[] = [];
  for (const cb of callbacks) {
    const id = `CB_${safe(cb)}`;
    cbIds.push(id);
    lines.push(`  ${id}["${cb}"]`);
    lines.push(`  S -->|executes| ${id}`);
  }

  // Topic publishers (shown when full I/O)
  const pubIds: string[] = [];
  if (showFull) {
    for (const t of node.topics.filter((t) => t.direction === "pub")) {
      const id = `P_${safe(t.topic)}`;
      pubIds.push(id);
      lines.push(`  ${id}["${trunc(t.topic)} : ${trunc(t.msgType, 24)}"]`);
      lines.push(`  S -->|pub| ${id}`);
    }
  }

  // Topic subscribers (shown when full or limited I/O)
  const subIds: string[] = [];
  if (showFull || showLimited) {
    for (const t of node.topics.filter((t) => t.direction === "sub")) {
      const id = `B_${safe(t.topic)}`;
      subIds.push(id);
      lines.push(`  ${id}["${trunc(t.topic)} : ${trunc(t.msgType, 24)}"]`);
      lines.push(`  ${id} -->|sub| S`);
    }
  }

  // Services / clients
  const srvIds: string[] = [];
  if (showFull || showLimited) {
    for (const svc of node.services) {
      const id = `V_${safe(svc.service)}`;
      srvIds.push(id);
      lines.push(`  ${id}["${trunc(svc.service)} : ${trunc(svc.srvType, 24)}"]`);
      if (svc.role === "server") lines.push(`  ${id} -.->|serve| S`);
      else lines.push(`  S -.->|call| ${id}`);
    }
  }

  // Actions
  const actIds: string[] = [];
  if (showFull) {
    for (const act of node.actions) {
      const id = `A_${safe(act.action)}`;
      actIds.push(id);
      lines.push(`  ${id}["${trunc(act.action)} : ${trunc(act.actionType, 24)}"]`);
      if (act.role === "server") lines.push(`  ${id} ==>|action| S`);
      else lines.push(`  S ==>|call| ${id}`);
    }
  }

  if (cbIds.length === 0 && pubIds.length === 0 && subIds.length === 0 && srvIds.length === 0 && actIds.length === 0) {
    lines.push(`  S --> NONE["No active I/O in this state"]`);
  }

  // Styles
  lines.push(`  classDef state fill:#1e3a5f,stroke:#3b82f6,color:#e2e8f0,font-weight:bold`);
  lines.push(`  classDef cb fill:#292000,stroke:#f59e0b,color:#fcd34d`);
  lines.push(`  classDef pub fill:#164e63,stroke:#67e8f9,color:#67e8f9`);
  lines.push(`  classDef sub fill:#052e16,stroke:#86efac,color:#86efac`);
  lines.push(`  classDef srv fill:#2e1065,stroke:#d8b4fe,color:#d8b4fe`);
  lines.push(`  classDef act fill:#1a1000,stroke:#fcd34d,color:#fcd34d`);
  lines.push(`  class S state`);
  if (cbIds.length > 0) lines.push(`  class ${cbIds.join(",")} cb`);
  if (pubIds.length > 0) lines.push(`  class ${pubIds.join(",")} pub`);
  if (subIds.length > 0) lines.push(`  class ${subIds.join(",")} sub`);
  if (srvIds.length > 0) lines.push(`  class ${srvIds.join(",")} srv`);
  if (actIds.length > 0) lines.push(`  class ${actIds.join(",")} act`);

  return lines.join("\n");
}


function reachableStates(transitions: Transition[], start: string, depth: number): Set<string> {
  const visited = new Set<string>([start]);
  let frontier = new Set<string>([start]);
  for (let i = 0; i < depth; i++) {
    const next = new Set<string>();
    for (const s of frontier) {
      for (const t of transitions) {
        if (t.from === s && !visited.has(t.to)) next.add(t.to);
        if (t.to === s && !visited.has(t.from)) next.add(t.from);
      }
    }
    for (const s of next) visited.add(s);
    frontier = next;
    if (next.size === 0) break;
  }
  return visited;
}

// ── Mermaid code builder ──────────────────────────────────────────────────────
function buildMermaid(transitions: Transition[], highlighted?: string, focusDepth?: number): string {
  let active = transitions;
  if (highlighted && focusDepth !== undefined) {
    const reachable = reachableStates(transitions, highlighted, focusDepth);
    active = transitions.filter((t) => reachable.has(t.from) && reachable.has(t.to));
  }
  const lines = active.map((t) => `  ${t.from.replace(/\s/g, "_")} --> ${t.to.replace(/\s/g, "_")} : ${t.trigger}`);
  let code = "stateDiagram-v2\n";
  if (!active.some((t) => t.from === "[*]" || t.to === "[*]")) {
    const firstState = active[0]?.from ?? "Unknown";
    code += `  [*] --> ${firstState.replace(/\s/g, "_")}\n`;
  }
  code += lines.join("\n");
  return code;
}

function buildSequenceDiagram(nodes: RosNode[]): string {
  const lines: string[] = ["sequenceDiagram"];
  for (const n of nodes) lines.push(`  participant ${n.nodeName.replace(/\//g, "_")}`);
  for (const pub of nodes) {
    for (const t of pub.topics.filter((tp) => tp.direction === "pub")) {
      const subs = nodes.filter((n) => n.topics.some((tp) => tp.direction === "sub" && tp.topic === t.topic));
      for (const sub of subs) {
        lines.push(`  ${pub.nodeName.replace(/\//g, "_")}->>${sub.nodeName.replace(/\//g, "_")}: ${t.topic}`);
      }
    }
  }
  return lines.join("\n");
}

// ── Mermaid renderer ──────────────────────────────────────────────────────────
let _mermaidId = 0;
function MermaidDiagram({ code }: { code: string }) {
  const ref = useRef<HTMLDivElement>(null);
  const [error, setError] = useState<string | null>(null);

  useEffect(() => {
    if (!ref.current) return;
    const id = `mermaid-${++_mermaidId}`;
    setError(null);
    mermaid.render(id, code)
      .then(({ svg }) => { if (ref.current) ref.current.innerHTML = svg; })
      .catch((e) => setError(String(e)));
  }, [code]);

  if (error)
    return <pre style={{ color: "#f87171", fontSize: 11, background: "#1c0a0a", padding: 12, borderRadius: 6 }}>{error}</pre>;
  return <div ref={ref} style={{ background: "#0f172a", borderRadius: 8, padding: 12 }} />;
}

// ── State detail panel ────────────────────────────────────────────────────────
function StateDetailPanel({ stateName, node, transitions }: { stateName: string; node?: RosNode; transitions: Transition[] }) {
  const [filter, setFilter] = useState("");
  const info: StateInfo | undefined = LIFECYCLE_STATE_INFO[stateName];
  const transOut = transitions.filter((t) => t.from === stateName || t.from === stateName.replace(/\s/g, "_"));
  const transIn = transitions.filter((t) => t.to === stateName || t.to === stateName.replace(/\s/g, "_"));

  const activityColor = { none: "#ef4444", limited: "#f59e0b", full: "#10b981" }[info?.topicActivity ?? "none"];
  const activityLabel = { none: "No I/O", limited: "Limited I/O", full: "Full I/O" }[info?.topicActivity ?? "none"];

  const q = filter.toLowerCase();

  // Active topics for this state
  const activeTopics = (node?.topics ?? []).filter((t) => {
    if (info) {
      if (info.topicActivity === "none") return false;
      if (info.topicActivity === "limited" && t.direction !== "sub") return false;
    }
    return !q || t.topic.toLowerCase().includes(q) || t.msgType.toLowerCase().includes(q);
  });

  // Active services for this state
  const activeServices = (node?.services ?? []).filter((s) => {
    if (info?.topicActivity === "none") return false;
    return !q || s.service.toLowerCase().includes(q) || s.srvType.toLowerCase().includes(q);
  });

  // Active actions for this state
  const activeActions = (node?.actions ?? []).filter((a) => {
    if (info?.topicActivity !== "full") return false;
    return !q || a.action.toLowerCase().includes(q) || a.actionType.toLowerCase().includes(q);
  });

  const hasNodeIO = node && (activeTopics.length + activeServices.length + activeActions.length) > 0;

  const pill = (label: string, color: string, bg: string) => (
    <span key={label} style={{ background: bg, color, borderRadius: 4, padding: "1px 7px", fontSize: 11, fontFamily: "monospace" }}>{label}</span>
  );

  return (
    <div style={{ padding: 16, borderLeft: "1px solid #1e293b", width: 300, flexShrink: 0, overflowY: "auto", background: "#080e1a" }}>
      <div style={{ fontWeight: 700, fontSize: 14, color: "#e2e8f0", marginBottom: 4 }}>{stateName}</div>
      {info && (
        <div style={{ fontSize: 11, color: "#94a3b8", marginBottom: 10, lineHeight: 1.5 }}>{info.description}</div>
      )}
      {info?.note && (
        <div style={{ fontSize: 11, color: "#60a5fa", background: "#0f1f3a", borderRadius: 4, padding: "4px 8px", marginBottom: 10 }}>
          ℹ {info.note}
        </div>
      )}
      {info && (
        <div style={{ display: "flex", alignItems: "center", gap: 6, marginBottom: 12 }}>
          <span style={{ width: 8, height: 8, borderRadius: "50%", background: activityColor, display: "inline-block" }} />
          <span style={{ fontSize: 11, color: activityColor }}>{activityLabel}</span>
        </div>
      )}

      {info?.callbacks && info.callbacks.length > 0 && (
        <div style={{ marginBottom: 12 }}>
          <div style={{ fontSize: 10, fontWeight: 600, color: "#64748b", textTransform: "uppercase", letterSpacing: "0.05em", marginBottom: 4 }}>Active Callbacks</div>
          <div style={{ display: "flex", flexWrap: "wrap", gap: 4 }}>
            {info.callbacks.map((cb) => pill(cb, "#fcd34d", "#292000"))}
          </div>
        </div>
      )}

      {transOut.length > 0 && (
        <div style={{ marginBottom: 12 }}>
          <div style={{ fontSize: 10, fontWeight: 600, color: "#64748b", textTransform: "uppercase", letterSpacing: "0.05em", marginBottom: 6 }}>Transitions OUT</div>
          {transOut.map((t, i) => (
            <div key={i} style={{ display: "flex", alignItems: "center", gap: 6, marginBottom: 4, fontSize: 11 }}>
              <span style={{ color: "#10b981", fontFamily: "monospace", flex: 1, overflow: "hidden", textOverflow: "ellipsis", whiteSpace: "nowrap" }}>{t.trigger}</span>
              <span style={{ color: "#475569" }}>→</span>
              <span style={{ color: "#60a5fa", fontFamily: "monospace" }}>{t.to}</span>
            </div>
          ))}
        </div>
      )}

      {transIn.length > 0 && (
        <div style={{ marginBottom: 12 }}>
          <div style={{ fontSize: 10, fontWeight: 600, color: "#64748b", textTransform: "uppercase", letterSpacing: "0.05em", marginBottom: 6 }}>Transitions IN</div>
          {transIn.map((t, i) => (
            <div key={i} style={{ display: "flex", alignItems: "center", gap: 6, marginBottom: 4, fontSize: 11 }}>
              <span style={{ color: "#a78bfa", fontFamily: "monospace" }}>{t.from}</span>
              <span style={{ color: "#475569" }}>→</span>
              <span style={{ color: "#475569", fontFamily: "monospace", flex: 1 }}>{t.trigger}</span>
            </div>
          ))}
        </div>
      )}

      {/* ── Node I/O section (only when a node is linked) ─────────────── */}
      {node && (
        <>
          <div style={{ fontSize: 10, fontWeight: 600, color: "#64748b", textTransform: "uppercase", letterSpacing: "0.05em", marginBottom: 6, marginTop: 4 }}>
            Node I/O in this state
          </div>
          {/* Search filter */}
          <input
            value={filter}
            onChange={(e) => setFilter(e.target.value)}
            placeholder="Filter topics / services…"
            style={{
              width: "100%", boxSizing: "border-box", marginBottom: 10,
              background: "#0f172a", border: "1px solid #1e293b",
              borderRadius: 4, padding: "4px 8px", color: "#e2e8f0", fontSize: 11, outline: "none",
            }}
          />

          {!hasNodeIO && (
            <div style={{ fontSize: 11, color: "#334155", fontStyle: "italic" }}>
              {filter ? "No matches." : "No I/O active in this state."}
            </div>
          )}

          {activeTopics.length > 0 && (
            <div style={{ marginBottom: 10 }}>
              <div style={{ fontSize: 10, color: "#475569", marginBottom: 4 }}>
                Topics {info?.topicActivity === "limited" ? "(subs only)" : ""}
              </div>
              {activeTopics.map((t, i) => (
                <div key={i} style={{ fontSize: 11, marginBottom: 3, display: "flex", gap: 6, alignItems: "flex-start" }}>
                  <span style={{ color: t.direction === "pub" ? "#67e8f9" : "#86efac", fontFamily: "monospace", minWidth: 26, fontSize: 9, paddingTop: 2 }}>
                    {t.direction === "pub" ? "PUB" : "SUB"}
                  </span>
                  <div>
                    <div style={{ color: "#e2e8f0", fontFamily: "monospace" }}>{t.topic}</div>
                    <div style={{ color: "#475569", fontSize: 10 }}>{t.msgType}</div>
                  </div>
                </div>
              ))}
            </div>
          )}

          {activeServices.length > 0 && (
            <div style={{ marginBottom: 10 }}>
              <div style={{ fontSize: 10, color: "#475569", marginBottom: 4 }}>Services</div>
              {activeServices.map((s, i) => (
                <div key={i} style={{ fontSize: 11, marginBottom: 3, display: "flex", gap: 6, alignItems: "flex-start" }}>
                  <span style={{ color: "#d8b4fe", fontFamily: "monospace", minWidth: 26, fontSize: 9, paddingTop: 2 }}>
                    {s.role === "server" ? "SRV" : "CLI"}
                  </span>
                  <div>
                    <div style={{ color: "#e2e8f0", fontFamily: "monospace" }}>{s.service}</div>
                    <div style={{ color: "#475569", fontSize: 10 }}>{s.srvType}</div>
                  </div>
                </div>
              ))}
            </div>
          )}

          {activeActions.length > 0 && (
            <div>
              <div style={{ fontSize: 10, color: "#475569", marginBottom: 4 }}>Actions</div>
              {activeActions.map((a, i) => (
                <div key={i} style={{ fontSize: 11, marginBottom: 3, display: "flex", gap: 6, alignItems: "flex-start" }}>
                  <span style={{ color: "#fcd34d", fontFamily: "monospace", minWidth: 26, fontSize: 9, paddingTop: 2 }}>
                    {a.role === "server" ? "ACT" : "CLI"}
                  </span>
                  <div>
                    <div style={{ color: "#e2e8f0", fontFamily: "monospace" }}>{a.action}</div>
                    <div style={{ color: "#475569", fontSize: 10 }}>{a.actionType}</div>
                  </div>
                </div>
              ))}
            </div>
          )}
        </>
      )}
    </div>
  );
}

// ── Main view ─────────────────────────────────────────────────────────────────
type DiagramMode = "ros2-lifecycle" | "node-state" | "topic-sequence";

export function LifecycleView() {
  const { data, selectedLifecycleNode, setSelectedLifecycleNode } = useStore();
  const [mode, setMode] = useState<DiagramMode>("ros2-lifecycle");
  const [selectedState, setSelectedState] = useState<string | null>(null);
  const [focusDepth, setFocusDepth] = useState<number>(0); // 0 = show all

  if (!data) return <div style={{ padding: 40, color: "#94a3b8" }}>No workspace data loaded.</div>;

  const selectedNode = data.nodes.find((n) => n.id === selectedLifecycleNode);

  const nodeTransitions = useMemo(
    () => (selectedNode ? getNodeTransitions(selectedNode) : []),
    [selectedNode]
  );

  // All states extracted from current transitions
  const stateNames = useMemo(() => {
    const src = mode === "ros2-lifecycle" ? ROS2_TRANSITIONS : nodeTransitions;
    return Array.from(new Set(src.flatMap((t) => [t.from, t.to]).filter((s) => s !== "[*]")));
  }, [mode, nodeTransitions]);

  const diagramCode = useMemo(() => {
    if (mode === "ros2-lifecycle") {
      return buildMermaid(ROS2_TRANSITIONS, selectedState ?? undefined, focusDepth > 0 ? focusDepth : undefined);
    }
    if (mode === "node-state") {
      if (!selectedNode) return "stateDiagram-v2\n  [*] --> SelectANode : select a node from the sidebar";
      // State selected → show interior I/O diagram for that state
      if (selectedState) return buildStateInteriorDiagram(selectedNode, selectedState);
      // No state selected → show full state machine, with optional focus around a hovered state
      return buildMermaid(nodeTransitions, undefined, focusDepth > 0 ? focusDepth : undefined);
    }
    return buildSequenceDiagram(data.nodes);
  }, [mode, selectedNode, nodeTransitions, selectedState, focusDepth, data.nodes]);

  const tabs: { key: DiagramMode; label: string }[] = [
    { key: "ros2-lifecycle", label: "ROS2 Lifecycle Standard" },
    { key: "node-state", label: "Node State Machine" },
    { key: "topic-sequence", label: "Topic Sequence Flow" },
  ];

  const showStatePanel = (mode === "ros2-lifecycle" || mode === "node-state") && selectedState;
  const currentTransitions = mode === "ros2-lifecycle" ? ROS2_TRANSITIONS : nodeTransitions;

  return (
    <div style={{ display: "flex", height: "100%", overflow: "hidden" }}>
      {/* ── Left sidebar ──────────────────────────────────────────────────── */}
      <div style={{ width: 220, borderRight: "1px solid #1e293b", overflowY: "auto", padding: "16px 0", flexShrink: 0 }}>
        <div style={sectionLabel}>Diagram type</div>
        {tabs.map((t) => (
          <div key={t.key} onClick={() => { setMode(t.key); setSelectedState(null); }}
            style={{ padding: "8px 16px", cursor: "pointer", fontSize: 12,
              color: mode === t.key ? "#60a5fa" : "#94a3b8",
              background: mode === t.key ? "#1e3a5f" : "transparent",
              borderLeft: mode === t.key ? "3px solid #3b82f6" : "3px solid transparent" }}>
            {t.label}
          </div>
        ))}

        {/* Node selector — node-state mode */}
        {mode === "node-state" && (
          <>
            <div style={sectionLabel}>Select Node</div>
            {data.nodes.map((n) => (
              <div key={n.id}
                onClick={() => { setSelectedLifecycleNode(n.id === selectedLifecycleNode ? null : n.id); setSelectedState(null); }}
                style={{ padding: "6px 16px", cursor: "pointer", fontSize: 12,
                  color: n.id === selectedLifecycleNode ? "#34d399" : "#94a3b8",
                  background: n.id === selectedLifecycleNode ? "#0d2a1e" : "transparent",
                  borderLeft: n.id === selectedLifecycleNode ? "3px solid #10b981" : "3px solid transparent" }}>
                <div style={{ fontWeight: 600 }}>{n.nodeName}</div>
                <div style={{ color: "#475569", fontSize: 10 }}>{n.package}</div>
                {!n.lifecycleNode && (
                  <div style={{ color: "#57534e", fontSize: 9, marginTop: 1 }}>inferred</div>
                )}
              </div>
            ))}
          </>
        )}

        {/* State selector */}
        {(mode === "ros2-lifecycle" || (mode === "node-state" && selectedNode)) && stateNames.length > 0 && !selectedState && (
          <>
            <div style={sectionLabel}>Select State</div>
            {stateNames.map((s) => (
              <div key={s}
                onClick={() => setSelectedState(s)}
                style={{ padding: "5px 16px", cursor: "pointer", fontSize: 12,
                  color: "#94a3b8",
                  background: "transparent",
                  borderLeft: "3px solid transparent" }}>
                {s}
              </div>
            ))}
          </>
        )}

        {/* When a state is selected: show it highlighted + back link */}
        {selectedState && (
          <>
            <div style={sectionLabel}>Selected State</div>
            <div style={{ padding: "5px 16px 2px", fontSize: 12, color: "#fcd34d", background: "#1c1500", borderLeft: "3px solid #f59e0b", fontWeight: 600 }}>
              {selectedState}
            </div>
            <div
              onClick={() => setSelectedState(null)}
              style={{ padding: "4px 16px 8px", fontSize: 11, color: "#3b82f6", cursor: "pointer" }}
            >
              ← back to all states
            </div>
          </>
        )}

        {/* Focus depth — available in ros2-lifecycle mode (always) and in node-state when no state is selected for drill-in */}
        {(mode === "ros2-lifecycle" || (mode === "node-state" && selectedNode && !selectedState)) && (
          <>
            <div style={sectionLabel}>Focus depth</div>
            <div style={{ padding: "4px 16px 8px" }}>
              <div style={{ fontSize: 11, color: "#64748b", marginBottom: 6 }}>
                {focusDepth === 0 ? "All transitions visible" : `±${focusDepth} hop${focusDepth > 1 ? "s" : ""} from selected`}
              </div>
              <div style={{ display: "flex", gap: 4, flexWrap: "wrap" }}>
                {[0, 1, 2, 3, 4].map((d) => (
                  <button key={d}
                    onClick={() => setFocusDepth(d)}
                    style={{ padding: "3px 10px", borderRadius: 4, border: `1px solid ${focusDepth === d ? "#f59e0b" : "#334155"}`,
                      background: focusDepth === d ? "#1c1500" : "#0f172a", color: focusDepth === d ? "#fcd34d" : "#64748b",
                      cursor: "pointer", fontSize: 11 }}>
                    {d === 0 ? "all" : d}
                  </button>
                ))}
              </div>
            </div>
          </>
        )}
      </div>

      {/* ── Diagram area ──────────────────────────────────────────────────── */}
      <div style={{ flex: 1, overflowY: "auto", padding: 24, minWidth: 0 }}>
        <div style={{ marginBottom: 12, fontSize: 12, color: "#64748b" }}>
          {mode === "ros2-lifecycle" && !selectedState && "Standard ROS2 Managed Node Lifecycle (rclcpp_lifecycle)"}
          {mode === "ros2-lifecycle" && selectedState && (
            <span>Lifecycle reference · <strong style={{ color: "#fcd34d" }}>{selectedState}</strong> — select a state to highlight</span>
          )}
          {mode === "node-state" && !selectedNode && "Select a node from the sidebar to view its state machine."}
          {mode === "node-state" && selectedNode && !selectedState && (
            <span>
              <strong style={{ color: "#34d399" }}>{selectedNode.nodeName}</strong>
              {!selectedNode.lifecycleNode && <span style={{ color: "#57534e" }}> (inferred)</span>}
              {" — click a state in the sidebar to inspect its active I/O"}
            </span>
          )}
          {mode === "node-state" && selectedNode && selectedState && (
            <span>
              <strong style={{ color: "#34d399" }}>{selectedNode.nodeName}</strong>
              {" → state "}
              <strong style={{ color: "#fcd34d" }}>{selectedState}</strong>
              {" · active methods & I/O shown below"}
            </span>
          )}
          {mode === "topic-sequence" && "Message flow between all nodes."}
        </div>
        <MermaidDiagram key={`${mode}-${selectedLifecycleNode}-${selectedState}-${focusDepth}`} code={diagramCode} />
        {!selectedState && focusDepth > 0 && (
          <div style={{ marginTop: 8, fontSize: 11, color: "#64748b" }}>
            Showing states within {focusDepth} hop{focusDepth > 1 ? "s" : ""} of all visible states.{" "}
            <span style={{ color: "#3b82f6", cursor: "pointer" }} onClick={() => setFocusDepth(0)}>Show all</span>
          </div>
        )}
      </div>

      {/* ── State detail panel ────────────────────────────────────────────── */}
      {showStatePanel && (
        <StateDetailPanel
          stateName={selectedState!}
          node={mode === "node-state" ? selectedNode : undefined}
          transitions={currentTransitions}
        />
      )}
    </div>
  );
}

const sectionLabel: React.CSSProperties = {
  padding: "14px 12px 6px",
  fontSize: 10,
  color: "#64748b",
  fontWeight: 700,
  textTransform: "uppercase",
  letterSpacing: "0.06em",
};
