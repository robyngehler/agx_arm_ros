# Sprint 2 Decisions And Open Questions

## Remaining Open Questions

### First Real OmniHand Backend

- Should the first non-mock backend target direct SDK access only, or should the repo keep an optional vendor-ROS adapter behind the same bridge surface?

### Long-Term Hand Command Surface

- Should `control/omnihand/joint_trajectory` remain a compatibility surface only, or should the repo promote a more explicit action or controller contract once the non-mock backend exists?

### Package Boundary After Real Backend Validation

- If the real backend adds more dependencies or rebuild churn, does the bridge still belong inside `agx_arm_ctrl`, or does a dedicated package become materially clearer only then?

### Runtime Graph Validation Evidence

- Once a full mock or live launch is run again, should the repo promote a captured runtime graph or launch trace into a stable doc, or keep the current diagrams source-derived only until the next validation pass?