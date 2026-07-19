# OmniHand Active Joint Map

status: LEGACY_O10_VENDOR_BASELINE
runtime_verification: HISTORICAL_ONLY
last_updated: 2026-05-12
source_documents:
- `vendor/OmniHand-Pro-2025/document/en/API_CPP.md`
- `vendor/OmniHand-Pro-2025/document/en/API_PYTHON.md`

Current stable repo runtime note:

- the current hardware-backed baseline uses the OmniHand Pro O12 model with 12 active joints
- this document is kept only as the earlier O10 vendor-declared mapping reference that informed the
	first wrapper-first design work

## Purpose

This document records the earlier vendor-declared O10 active-joint order and limits that were used
as the first mapping reference during early wrapper design.

It is a historical mapping reference only. It is not the current stable runtime contract and it is
not a claim that every value below was verified on the current O12 hardware path.

## Finger Grouping

- joints `1-3`: thumb
- joints `4-5`: index finger
- joint `6`: middle finger
- joints `7-8`: ring finger
- joints `9-10`: little finger

## Right Hand Active-Joint Order

| Index | Joint Name | Min Rad | Max Rad | Min Deg | Max Deg | Velocity Limit Rad/s |
| --- | --- | --- | --- | --- | --- | --- |
| 1 | `R_thumb_roll_joint` | -0.17453292519943295 | 0.8726646259971648 | -10 | 50 | 0.164 |
| 2 | `R_thumb_abad_joint` | -1.7453292519943295 | 0 | -100 | 0 | 0.164 |
| 3 | `R_thumb_mcp_joint` | 0 | 0.8552113334772214 | 0 | 49 | 0.308 |
| 4 | `R_index_abad_joint` | -0.20943951023931953 | 0 | -12 | 0 | 0.164 |
| 5 | `R_index_pip_joint` | 0 | 1.5707963267948966 | 0 | 90 | 0.308 |
| 6 | `R_middle_pip_joint` | 0 | 1.5707963267948966 | 0 | 90 | 0.308 |
| 7 | `R_ring_abad_joint` | 0 | 0.17453292519943295 | 0 | 10 | 0.164 |
| 8 | `R_ring_pip_joint` | 0 | 1.5707963267948966 | 0 | 90 | 0.308 |
| 9 | `R_pinky_abad_joint` | 0 | 0.17453292519943295 | 0 | 10 | 0.164 |
| 10 | `R_pinky_pip_joint` | 0 | 1.5707963267948966 | 0 | 90 | 0.308 |

## Left Hand Active-Joint Order

| Index | Joint Name | Min Rad | Max Rad | Min Deg | Max Deg | Velocity Limit Rad/s |
| --- | --- | --- | --- | --- | --- | --- |
| 1 | `L_thumb_roll_joint` | -0.8726646259971648 | 0.17453292519943295 | -50 | 10 | 0.164 |
| 2 | `L_thumb_abad_joint` | 0 | 1.7453292519943295 | 0 | 100 | 0.164 |
| 3 | `L_thumb_mcp_joint` | -0.8552113334772214 | 0 | -49 | 0 | 0.308 |
| 4 | `L_index_abad_joint` | 0 | 0.20943951023931953 | 0 | 12 | 0.164 |
| 5 | `L_index_pip_joint` | 0 | 1.5707963267948966 | 0 | 90 | 0.308 |
| 6 | `L_middle_pip_joint` | 0 | 1.5707963267948966 | 0 | 90 | 0.308 |
| 7 | `L_ring_abad_joint` | -0.17453292519943295 | 0 | -10 | 0 | 0.164 |
| 8 | `L_ring_pip_joint` | 0 | 1.5707963267948966 | 0 | 90 | 0.308 |
| 9 | `L_pinky_abad_joint` | -0.17453292519943295 | 0 | -10 | 0 | 0.164 |
| 10 | `L_pinky_pip_joint` | 0 | 1.5707963267948966 | 0 | 90 | 0.308 |

## Recommended First Motion Candidate

For the first controlled command-response loop, prefer one of the PIP joints with a small delta, because the limits are simple and the vendor examples already use active-joint angle commands in aggregate.

Recommended defaults for the repo smoke test:

- hand: left
- joint index: `5`
- joint name: `L_index_pip_joint`
- delta: `+0.05 rad`
- behavior: apply once, read back, then restore the starting angle vector

## Runtime Verification Notes To Fill In Later

When hardware validation becomes possible, append the following for the tested hand:

- actual adapter model
- host architecture
- confirmed device ID
- whether the runtime order matches the vendor-declared index table exactly
- any per-joint sign or limit differences observed in practice