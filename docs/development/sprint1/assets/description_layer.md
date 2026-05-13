```mermaid
flowchart TD
    A[agx_arm_urdf asset tree] --> B[display.launch.py]
    A --> C[display_control.launch.py]
    A --> D[agx_arm.urdf.xacro in MoveIt]

    B --> E[agx_arm_description.urdf.xacro]
    E --> F[robot_description]
    F --> G[robot_state_publisher]
    F --> H[RViz]

    C --> I[direct URDF or Xacro selection]
    I --> J[robot_description]
    J --> K[robot_state_publisher]
    J --> L[RViz]
    C --> M[joint_state_publisher or GUI]
    C --> N[tcp_link static TF]

    subgraph Active Purpose
      B1[display.launch.py]
      B2[generic model and sim view]
      C1[display_control.launch.py]
      C2[control and RViz compatibility layer]
    end

    B1 --> B2
    C1 --> C2
    ```