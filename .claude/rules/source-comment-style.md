# Source Comment Style

*Use when writing or editing comments and docstrings under `src/`. Covers how much
context belongs in the code and where the rest goes.*

A comment says **what the code does and why it exists**. Short.

## The rule

State the current behaviour and its reason. Do not narrate how the code got
there.

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

## Do not write

- **History.** "used to be one", "this used to warn and then return True
  anyway", "before 4D this passed unconditionally". The diff and `git log` hold
  that.
- **Incident retellings.** "observed on hardware 2026-07-24, left arm", measured
  numbers, and the story of the bug that motivated the code. Those belong in
  `docs/sprint*/errors_and_fixes.md` and the sprint reference notes.
- **Plot twists and rhetoric.** "which is the race this structure exists to
  remove", "the one thing it cannot do", "and that is the point".
- **Restating the diff.** A comment that lists what changed is a changelog entry
  in the wrong file.

## Do write

- one line on what the function or block does, when the name does not already
  say it
- the reason a non-obvious choice was made, in a clause — not a paragraph
- a constraint a future editor would otherwise break
- a pointer to the doc that carries the detail, when the detail matters:
  `see docs/sprint_refactor/reference/sdk_latency_budget.md`

## Length

A module docstring may set context in a few lines. A function docstring is
normally one to three lines. A block comment is one or two. If a rationale needs
more, it is a documentation change with a pointer from the code, not a longer
comment.

## Where the removed material goes

| Content | Home |
| --- | --- |
| why a bug happened and how it was found | `docs/sprint*/errors_and_fixes.md` |
| measurements, budgets, hardware evidence | `docs/sprint*/reference/` |
| contract and interface decisions | `docs/assets/`, `AGENTS.md`, `.claude/rules/` |
| what changed in this commit and why | the commit message |

## Not covered by this rule

Commit messages keep the form in `.claude/skills/commit-quality/SKILL.md`: they
explain the system-level change, why it was needed, and its consequence. That is
the place for the narrative this rule keeps out of the source.
