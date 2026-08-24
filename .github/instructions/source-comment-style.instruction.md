---
description: "Use when writing or editing comments and docstrings under src/ in agx_arm_ros. Covers how much context belongs in the code and where the rest goes."
applyTo: "src/**"
---

# Source Comment Style

A comment states **what the code does and the constraint that shaped it**. Short.

## Tone: describe, do not narrate

A comment is a description of the code as it stands, not an account of how it got there or what it means to the author. Name the behaviour, name the constraint, give the number where a number is the reason. Stop.

**Do not:**

- **Narrate history.** "used to be one", "this used to warn and then return True anyway", "before 4D this passed unconditionally". `git log` holds that.
- **Retell an incident.** "observed on hardware 2026-07-24, left arm, which is when the runaway happened". Write the constraint the incident established, not the incident.
- **Explain what a mistake really was.** "deriving the first from the second is what deadlocked a silent arm: the command that restores feedback was refused for want of the feedback it exists to restore." Write the three facts.
- **Chain a consequence into a story.** "which makes the planner able to plan what the controller would refuse, and the tracking error that follows trips the per-joint hold" is three hops. One clause: "the planner may exceed the controller's clamp."
- **Editorialise a design choice.** "blunt, local, and one number an operator turns rather than a fit parameter" → "moving-average window, width in seconds".
- **Dramatise.** "which is the race this structure exists to remove", "the one thing it cannot do", "and that is the point", "precisely the state the hold exists to prevent".
- **Restate the diff.** A comment listing what changed is a changelog entry in the wrong file.

**Do:**

```python
# Three facts: a transport exists and can carry a command; feedback is
# advancing; the joints answered the last enable request.
```

not

```python
# Three facts that used to be one. A transport exists and can carry a command
# (transport_connected); feedback is advancing (control_ready, is_ok); the
# joints answered the last enable request (enable_flag). Deriving the first
# from the second is what deadlocked a silent arm: the command that restores
# feedback was refused for want of the feedback it exists to restore.
```

If a sentence could be deleted without losing a fact, delete it.

## Numbers

A measurement belongs in a comment when it **is** the reason for the code — a constant's value, a threshold, a bound a future editor would otherwise relax. Give it plainly, without the story around it:

- ✅ `# 1.3x overshoot at blend junctions, so the limits are corrected against the sampled peak`
- ✅ `# 109 rad/s² at the edges against 5.8 over the rest, so the window is reflected rather than shrunk`
- ❌ `# We first tried a fixed derating of 1.3 and it turned out that the real overshoot depends on the blend radius, which is what finally explained the spikes`

A measurement that explains *how a defect was found* is not a reason for the code. That goes in `docs/sprint*/errors_and_fixes.md`.

## Do write

- one line on what the function or block does, when the name does not already say it
- the reason a non-obvious choice was made, in a clause — not a paragraph
- a constraint a future editor would otherwise break
- a pointer to the doc that carries the detail: `see docs/sprint_refactor/reference/sdk_latency_budget.md`

## Length

A module docstring may set context in a few lines. A function docstring is normally one to three lines. A block comment is one or two. If a rationale needs more, it is a documentation change with a pointer from the code, not a longer comment.

## Where the removed material goes

| Content | Home |
| --- | --- |
| why a bug happened and how it was found | `docs/sprint*/errors_and_fixes.md` |
| measurements, budgets, hardware evidence | `docs/sprint*/reference/` |
| contract and interface decisions | `docs/assets/`, `AGENTS.md`, `.github/instructions/` |
| what changed in this commit and why | the commit message |

## Not covered by this rule

Commit messages keep the form in `.github/skills/commit-quality/SKILL.md`: the system-level change, why it was needed, its consequence. The same tone rule applies there.
