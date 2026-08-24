---
name: commit-quality
description: Write concise, outsider-readable commits that explain the system-level change, motivation, and impact without repeating implementation details from the diff. Consult before every commit and when squashing or rewriting commit history.
---

# Commit Quality

## Goal

A commit message must let an external reader understand:

1. **what changed at the system level;**
2. **why the change was necessary;**
3. **what consequence it has for the components, MVPs, or project plan.**

Implementation detail belongs in the diff, tests, and canonical docs—not in a miniature changelog inside the commit body.

## Commit structure

```text
<type>(<scope>): <system-level outcome>

<One or two sentences: what this attacks and why it mattered.>

- <key point — what was wrong, what now holds, or what follows from it>
- <key point>
- <key point>

Validation: <only the decisive checks>
```

Rules:

- Subject: imperative, specific, preferably ≤72 characters.
- Framing: one or two sentences, then **key points as bullets**. Normally 2–5 bullets.
- Each bullet is a **finding or a consequence**, not a task and not a file. If a bullet could be reordered with any other without loss, it is probably an inventory entry — cut it.
- Lead with the **meta-level outcome**, not filenames or function names.
- Mention only decisive validation, not every test module.
- Do not copy checklist entries, proposal sections, or implementation inventories.
- Do not narrate the full investigation history.
- Separate unrelated changes into separate commits.
- If the change alters a claim, matrix, gate, deadline, or contract, state that consequence explicitly and update the canonical docs in the same commit.
- **Never name co-authorship.** No `Co-Authored-By:` trailer, no tool or model attribution, no "generated with" line — in any commit message, amend, or PR body. The author is the repository owner; the message is about the change, not about who or what typed it.

## Tone: describe, do not narrate

A commit message is a record, not an account of how the work went. State what
changed, what it was before, and what was measured. Stop there.

**Do not:**

- **Explain what a mistake "really was."** "both were the same mistake: deriving
  a number instead of using the one that exists" is an interpretation. Write the
  two values instead.
- **Chain a consequence into a story.** "the planner could plan what the
  controller would refuse, and the tracking error that followed would trip the
  per-joint hold" is three hops of narration. One clause: "the planner could
  exceed the controller's clamp."
- **Editorialise a design choice.** "blunt, local, and one number an operator
  turns rather than a fit parameter" — the reader wants "moving-average window,
  width in seconds, set at playback."
- **Justify inside the message.** "deliberately independent, because tying them
  would make a replay's path depend on a number chosen for a different stage."
  The rationale belongs in the code comment or the doc. In the commit: "the
  window is separate from the spline smoothing used for the TOTG path."
- **Dramatise a defect.** "a spike next to each endpoint that the motion does
  not contain" → "a ~23 rad/s² spike at the second and second-to-last sample."
- **Say what a number means to you.** "the number that matters", "which is what
  finally separated", "bitter", "the tell". Give the number.

**Do:** name the thing, give the before and after value, give the measurement.
If a sentence could be deleted without losing a fact, delete it.

A plain list of what changed is not a "miniature changelog" — that rule is
about restating the diff file by file. Grouped, factual statements of behaviour
change are exactly right.

## Scopes

Use the scope already established in `git log`: the package or surface the change
belongs to — `ctrl`, `coordination`, `mit`, `omnihand`, `moveit`, `msgs`,
`description`, `can`, `teach`, `demo`, `setup`, `docs`, `config`. A sprint scope
(`docs(sprint_refactor)`) is right for documentation work; for code, name the
package, not the sprint.

## No acronym maze

This repository is dense with short references — phase numbers like `2A`, binding
constraints like `C1`, sprint surfaces, message and topic names. They are precise
**inside** the docs, where the reader is one click from the definition. In a
commit message they are opaque: a reader scanning `git log` has no index open.

**The rule: say it in plain words first, then reference.**

- ✅ "each device now has its own CAN bus, so the hand no longer waits for an arm window (C1)"
- ❌ "implements C1 per plan 2A, supersedes §3.3/§7"
- ✅ "the bridge derived its CAN interface from the arm's port and silently fell back to the right arm bus"
- ❌ "fixed resolve_can_interface per EF-2026-08-11 (see OQ rollout)"

Concretely:

- **Never write a sentence whose subject is an ID.** The subject is the thing
  that broke or changed.
- **At most a handful of IDs in a message**, each in parentheses after the plain
  description. A bullet that is only IDs is a lookup table, not a message.
- **Never chain references** (`C1/C2/C5`, `2A/2B/2C`, `§3.3/§7/§9`). If three
  things closed, say what the three things *were*.
- A **file path** is worth naming only when it *is* the point (a new entry
  point, a moved contract). Otherwise the diff already says it.

## What to include

Prefer:

- the problem or risk being removed;
- what actually went wrong, in one clause;
- the design decision taken;
- the resulting project-level capability or constraint;
- the most important validation result;
- any remaining limitation that affects interpretation.

Avoid:

- long file lists;
- line-by-line implementation summaries;
- every test name and count;
- repeated background already recorded in `errors_and_fixes.md`;
- claims such as "fully fixed" unless the relevant gate actually passed;
- "validated" for anything that only ran against mocks — this repository
  distinguishes unit, mock, and hardware evidence, and a commit that blurs them
  is worse than one that admits the gap.

## Hardware honesty

A commit touching CAN, timing, or motion states which level its evidence came
from. If the change was not exercised on hardware, the message says so in one
clause. `AGENTS.md` requires this of the work; it belongs in the record too.

## Examples

Bad — an inventory, and a reference maze:

```text
feat(ctrl): add worker queue, fix OQ-3, update C1/C2 docs, epoch field, validation, retry path, 12 tests, checklist ticks, mirror sync...
```

Good — findings first, references in support:

```text
feat(ctrl): give each arm one owner for its SDK session

Mode changes, motion callbacks and recovery all reached the vendor SDK from
different threads, so a precondition check could pass and be void by the time
the call landed.

- Every hardware operation for a side now goes through one worker, with
  emergency stop on a priority lane ahead of queued motion.
- Commands carry the device and unit generations they were issued under, so
  anything queued across an ownership change is dropped instead of arriving late.
- Feedback is taken once per acquisition cycle and shared, which removes the
  per-joint SDK reads that were starving the publish thread.

Validation: mock-level only — the epoch and queue behaviour are covered by
package tests, the CPU effect is not measured until the hardware baseline runs.
```

Bad — factually correct, but narrating and interpreting throughout:

```text
feat(retiming): one velocity limit, and a blunt replay that fits nothing

Two changes that belong together, because both were the same mistake: deriving
a number instead of using the one that exists.

The manufacturer specifies 180 deg/s on J1-J3 and 225 on J4-J7. The planner
config declared 5.0 rad/s and the MIT controller clamped at 2.0 — three numbers
for one quantity, where the planner could plan what the controller would refuse
and the tracking error that followed would trip the per-joint hold.
```

The same change, described:

```text
feat(retiming): adopt manufacturer joint velocity limits

joint_limits.yaml declared 5.0 rad/s and the MIT controller clamped at 2.0,
against a manufacturer maximum of 3.14 (J1-J3) and 3.93 (J4-J7). The planner
could therefore exceed the controller's clamp.

- Both configs now declare the manufacturer values.
- No acceleration is specified for these joints; 2.5 * v_max is used as a
  stand-in and marked as such in both files.

Validation: package tests. Not run on hardware — the raised clamp permits
faster commanded motion than before.
```

## Final check

Before committing, ask:

> Could an engineer unfamiliar with this session understand the purpose,
> consequence, and confidence level of this commit in under 20 seconds —
> **without opening a single doc?**

If not, shorten and raise the abstraction level.
