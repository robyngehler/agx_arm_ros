// ── ROS Workspace Data Model ─────────────────────────────────────────────────

export interface MsgDef {
  name: string;        // e.g. "AgxArmStatus"
  package: string;     // e.g. "agx_arm_msgs"
  kind: "msg" | "srv" | "action";
  fields: { name: string; type: string }[];
  filePath: string;
}

export interface TopicConnection {
  topic: string;
  msgType: string;
  direction: "pub" | "sub";
}

export interface ServiceConnection {
  service: string;
  srvType: string;
  role: "server" | "client";
}

export interface ActionConnection {
  action: string;
  actionType: string;
  role: "server" | "client";
}

export interface ParameterDef {
  name: string;
  type: string;
  default?: string;
  description?: string;
}

export interface RosNode {
  id: string;            // unique: "pkg/node_name"
  nodeName: string;
  package: string;
  filePath: string;
  topics: TopicConnection[];
  services: ServiceConnection[];
  actions: ActionConnection[];
  parameters: ParameterDef[];
  lifecycleNode: boolean;
  lifecycleStates?: LifecycleState[];
}

export interface LifecycleState {
  from: string;
  to: string;
  trigger: string;
}

export interface LaunchArg {
  name: string;
  default?: string;
  description?: string;
  choices?: string[];
}

export interface LaunchInclude {
  file: string;
  args?: Record<string, string>;
  condition?: string;
}

export interface LaunchNodeEntry {
  package: string;
  executable: string;
  name?: string;
  namespace?: string;
  parameters?: Record<string, string>;
  remappings?: [string, string][];
  condition?: string;
}

export interface LaunchFile {
  id: string;           // unique: "pkg/launch/filename"
  filePath: string;
  package: string;
  args: LaunchArg[];
  nodes: LaunchNodeEntry[];
  includes: LaunchInclude[];
}

export interface RosPackage {
  name: string;
  path: string;
  deps: string[];
  buildType: "ament_python" | "ament_cmake" | "cmake" | "unknown";
}

export interface RosEntryPoint {
  name: string;      // ros2 run <package> <name>
  module: string;    // python module:function
  package: string;
}

export interface WorkspaceData {
  root: string;
  scannedAt: string;
  packages: RosPackage[];
  nodes: RosNode[];
  launches: LaunchFile[];
  messages: MsgDef[];
  entryPoints: RosEntryPoint[];  // ros2 run console_scripts
}
