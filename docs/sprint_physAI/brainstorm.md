# This Document contains Ideas and possible Paths how to integrate Physical AI into the duo-body stack

## Models and their Roles:
#### Reasoning, Data Creation/Augmentation, Evaluation:
**Cosmos-Predict2.5-2B/robot/action-cond**
(possible other candidates: robot/multiview, robot/multiview-agibot, robot/gr00tdream-gr1)

Limits:
- No hard physics simulator,
- No realtime-controller
- Needs Camera-/Embodiment-Finetuning for our hardware stack 

#### Other Coordinators
**Gemini Robotics-ER**
```text
Scene + instruction
→ detect objects / affordances
→ decompose into subtasks
→ call skill API
→ monitor progress
```

#### Policy, Action Creation, Runtime:
**Isaac GR00T N1.7**

Pros:
- Fits dual-arm + gripper
- Relative ee-actions fit for skill-parametrization
- NVIDIA-stack, fits to Cosmos, Isaac Sim, Jetson/Thor, TensorRT.
- license unter Apache 2.0 

Limits:
- Early Access, limited stability-/support-guaranty
- Finetuning needed
- Contakt-/Force-Control needs Skill-Controller

#### Other VLA Policy Executors
- OpenVLA/OpenVLA-OFT
- openpi / π0.5
- RDT-1B
- SmolVLA


#### Physics Helpers
**PhysicsNeMo**
- Surrogate for contact-/material,
- Deformation modeling,
- Grip approximation,
- fast physically close digital-dwin components,
- possibly learned residual dynamics.


## About Latent Actions vs Motion Primitves/Skills
#### May Separate into:
- adressability/exposure — is there a handle, a coordinator knows/calls/parameterizes or is it an internal representation, only the policy emmits?
- origin — hand crafted (DMP, scripted controller) ↔ lerned (VAE-Codes, Latent-Action-Models)? learned still can be a semantic representation
- discret vs. continuess — Codebook/Library ↔ continuess Latent-Space, from which is sampeled?
Zeitliche Ausdehnung — Ein-Schritt-Transition ↔ zeitlich ausgedehntes Sub-Verhalten (Option/Skill) ↔ ganzer Subtask.
Selektionsmechanismus — diskretes Routing, kontinuierliche Konditionierung, oder MoE-Routing.