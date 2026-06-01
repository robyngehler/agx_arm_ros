import type { LaunchFile, LaunchNodeEntry, RosNode, ToolIntegrationKey, WorkspaceData } from "./types";

export const TOOL_INTEGRATION_OPTIONS: Array<{
  key: ToolIntegrationKey;
  label: string;
  description: string;
  accentColor: string;
}> = [
  {
    key: "moveit",
    label: "MoveIt",
    description: "Launch-derived move_group runtime nodes with FollowJointTrajectory and planning edges.",
    accentColor: "#10b981",
  },
  {
    key: "rviz",
    label: "RViz",
    description: "Launch-derived rviz2 runtime nodes with follow:=true-style visualization links.",
    accentColor: "#f59e0b",
  },
];

type ToolToggleState = Record<ToolIntegrationKey, boolean>;

type ToolRuntimeSeed = {
  key: ToolIntegrationKey;
  package: string;
  executable: string;
  nodeName: string;
  namespace?: string;
  launchPaths: Set<string>;
};

function normalizeNamespace(namespace?: string): string {
  const trimmed = String(namespace ?? "").trim();
  if (!trimmed) return "";
  return trimmed.startsWith("/") ? trimmed : `/${trimmed}`;
}

function qualifyName(namespace: string, resource: string): string {
  const normalizedNamespace = normalizeNamespace(namespace);
  const normalizedResource = resource.replace(/^\/+/, "");
  if (!normalizedNamespace) return normalizedResource;
  return `${normalizedNamespace}/${normalizedResource}`.replace(/^\//, "");
}

function stableToolId(key: ToolIntegrationKey, namespace: string, nodeName: string): string {
  const scope = normalizeNamespace(namespace).replace(/[^a-zA-Z0-9]+/g, "_") || "root";
  const name = nodeName.replace(/[^a-zA-Z0-9]+/g, "_");
  return `known_tool/${key}/${scope}/${name}`;
}

function matchesIntegration(key: ToolIntegrationKey, entry: LaunchNodeEntry): boolean {
  if (key === "moveit") {
    return entry.package === "moveit_ros_move_group" && entry.executable === "move_group";
  }
  return entry.package === "rviz2" && entry.executable === "rviz2";
}

function collectSeeds(data: WorkspaceData, toggles: ToolToggleState): ToolRuntimeSeed[] {
  const seeds = new Map<string, ToolRuntimeSeed>();

  const addSeed = (key: ToolIntegrationKey, launch: LaunchFile, entry: LaunchNodeEntry) => {
    const namespace = normalizeNamespace(entry.namespace);
    const nodeName = entry.name?.trim() || entry.executable;
    const runtimeId = `${key}:${namespace}:${nodeName}`;
    const existing = seeds.get(runtimeId);
    if (existing) {
      existing.launchPaths.add(launch.filePath);
      return;
    }
    seeds.set(runtimeId, {
      key,
      package: entry.package,
      executable: entry.executable,
      nodeName,
      namespace,
      launchPaths: new Set([launch.filePath]),
    });
  };

  for (const launch of data.launches) {
    for (const entry of launch.nodes) {
      if (toggles.moveit && matchesIntegration("moveit", entry)) addSeed("moveit", launch, entry);
      if (toggles.rviz && matchesIntegration("rviz", entry)) addSeed("rviz", launch, entry);
    }
  }

  return [...seeds.values()];
}

function buildMoveItNode(seed: ToolRuntimeSeed): RosNode {
  const namespace = seed.namespace ?? "";
  const followJointStatesTopic = qualifyName(namespace, "feedback/joint_states");
  const followAction = qualifyName(namespace, "arm_controller/follow_joint_trajectory");
  const moveAction = qualifyName(namespace, "move_action");
  const displayPlannedPath = qualifyName(namespace, "display_planned_path");

  return {
    id: stableToolId(seed.key, namespace, seed.nodeName),
    nodeName: seed.nodeName,
    package: seed.package,
    filePath: [...seed.launchPaths].sort()[0],
    topics: [
      { topic: followJointStatesTopic, msgType: "sensor_msgs/JointState", direction: "sub" },
      { topic: displayPlannedPath, msgType: "moveit_msgs/DisplayTrajectory", direction: "pub" },
    ],
    services: [],
    actions: [
      { action: followAction, actionType: "control_msgs/FollowJointTrajectory", role: "client" },
      { action: moveAction, actionType: "moveit_msgs/MoveGroup", role: "server" },
    ],
    parameters: [],
    lifecycleNode: false,
    sourceKind: "knownTool",
    integrationKey: "moveit",
    derivedFromLaunches: [...seed.launchPaths].sort(),
    notes: [
      "Launch-derived system tool node synthesized from moveit_ros_move_group/move_group entries.",
      "Assumes the repo default MIT execution path and links as an action client for arm_controller/follow_joint_trajectory.",
      "Assumes the default follow:=true path and subscribes to feedback/joint_states.",
    ],
  };
}

function buildRvizNode(seed: ToolRuntimeSeed): RosNode {
  const namespace = seed.namespace ?? "";
  const followJointStatesTopic = qualifyName(namespace, "feedback/joint_states");
  const displayPlannedPath = qualifyName(namespace, "display_planned_path");

  return {
    id: stableToolId(seed.key, namespace, seed.nodeName),
    nodeName: seed.nodeName,
    package: seed.package,
    filePath: [...seed.launchPaths].sort()[0],
    topics: [
      { topic: followJointStatesTopic, msgType: "sensor_msgs/JointState", direction: "sub" },
      { topic: displayPlannedPath, msgType: "moveit_msgs/DisplayTrajectory", direction: "sub" },
    ],
    services: [],
    actions: [],
    parameters: [],
    lifecycleNode: false,
    sourceKind: "knownTool",
    integrationKey: "rviz",
    derivedFromLaunches: [...seed.launchPaths].sort(),
    notes: [
      "Launch-derived system tool node synthesized from rviz2 launch entries.",
      "Assumes the repo default follow:=true path and visualizes feedback/joint_states.",
      "Subscribes to display_planned_path so MoveIt trajectory previews show up as first-class graph links.",
    ],
  };
}

export function buildKnownToolNodes(data: WorkspaceData, toggles: ToolToggleState): RosNode[] {
  const workspaceNodeIds = new Set(data.nodes.map((node) => node.id));
  return collectSeeds(data, toggles)
    .map((seed) => (seed.key === "moveit" ? buildMoveItNode(seed) : buildRvizNode(seed)))
    .filter((node) => !workspaceNodeIds.has(node.id));
}