import { useMemo, useState, useCallback, useEffect } from "react";
import {
  ReactFlow,
  Background,
  Controls,
  MiniMap,
  type Node,
  type Edge,
  MarkerType,
  Handle,
  Position,
  type NodeProps,
} from "@xyflow/react";
import "@xyflow/react/dist/style.css";
import Dagre from "@dagrejs/dagre";
import { Zap, Wrench, ArrowRight, ArrowLeft } from "lucide-react";
import { buildKnownToolNodes } from "../knownToolIntegrations";
import { useStore } from "../store";
import type { RosNode, MsgDef } from "../types";

type TopicBridgeVisibility = "internal" | "semantic" | "filtered" | "external";
type EndpointPresence = "semantic" | "filtered" | "external";
type SemanticAffinity = { services: string[]; actions: string[]; topics: string[] };

// ── Package colour map ────────────────────────────────────────────────────────
const PKG_COLORS: Record<string, string> = {
  agx_arm_ctrl: "#3b82f6",
  agx_arm_mit_controller: "#8b5cf6",
  agx_arm_moveit: "#10b981",
  agx_arm_description: "#f59e0b",
};

function pkgColor(pkg: string): string {
  return PKG_COLORS[pkg] ?? "#6b7280";
}

// ── Custom node card ──────────────────────────────────────────────────────────
function RosNodeCard({ data }: NodeProps) {
  const { node: n, isSelected = false } = data as { node: RosNode; isSelected?: boolean };
  const pubCount = n.topics.filter((t) => t.direction === "pub").length;
  const subCount = n.topics.filter((t) => t.direction === "sub").length;
  const srvCount = n.services.length;
  const actCount = n.actions.length;
  const border = pkgColor(n.package);

  return (
    <div
      style={{
        border: `2px solid ${isSelected ? "#ffffff" : border}`,
        borderRadius: 10,
        background: "#1e293b",
        color: "#e2e8f0",
        minWidth: 200,
        fontFamily: "monospace",
        fontSize: 12,
        boxShadow: isSelected ? `0 0 0 4px ${border}55, 0 0 28px ${border}44` : undefined,
        transition: "box-shadow 0.15s",
      }}
    >
      <Handle type="target" position={Position.Left} style={{ background: "#60a5fa" }} />
      <Handle type="source" position={Position.Right} style={{ background: "#34d399" }} />
      <div
        style={{
          background: border,
          borderRadius: "8px 8px 0 0",
          padding: "4px 10px",
          fontWeight: 700,
          fontSize: 11,
          color: "#fff",
        }}
      >
        {n.package}
      </div>
      <div style={{ padding: "6px 10px 8px" }}>
        <div style={{ fontWeight: 700, fontSize: 13, marginBottom: 4 }}>{n.nodeName}</div>
        <div style={{ display: "flex", gap: 6, flexWrap: "wrap" }}>
          {pubCount > 0 && (
            <span style={{ background: "#164e63", borderRadius: 4, padding: "1px 6px", color: "#67e8f9" }}>
              ▲ {pubCount} pub
            </span>
          )}
          {subCount > 0 && (
            <span style={{ background: "#14532d", borderRadius: 4, padding: "1px 6px", color: "#86efac" }}>
              ▼ {subCount} sub
            </span>
          )}
          {srvCount > 0 && (
            <span style={{ background: "#44337a", borderRadius: 4, padding: "1px 6px", color: "#d8b4fe" }}>
              ⚙ {srvCount} srv
            </span>
          )}
          {actCount > 0 && (
            <span style={{ background: "#78350f", borderRadius: 4, padding: "1px 6px", color: "#fcd34d" }}>
              ⚡ {actCount} act
            </span>
          )}
        </div>
      </div>
    </div>
  );
}

// ── Topic bridge node ─────────────────────────────────────────────────────────
function TopicBridgeCard({ data }: NodeProps) {
  const { topic, msgType, visibilityState = "internal" } = data as {
    topic: string;
    msgType: string;
    visibilityState?: TopicBridgeVisibility;
  };
  const isSemantic = visibilityState === "semantic";
  const isFiltered = visibilityState === "filtered";
  const isExternal = visibilityState === "external";
  return (
    <div
      style={{
        border: isExternal
          ? "1px dashed #6366f1"
          : isFiltered
            ? "1px dashed #38bdf8"
            : isSemantic
              ? "1px dashed #f59e0b"
              : "1px dashed #475569",
        borderRadius: 8,
        background: isExternal
          ? "#1e1b4b"
          : isFiltered
            ? "#0b1b2a"
            : isSemantic
              ? "#211406"
              : "#0f172a",
        color: "#94a3b8",
        padding: "4px 10px",
        fontFamily: "monospace",
        fontSize: 11,
        minWidth: 140,
        textAlign: "center",
      }}
    >
      <Handle type="target" position={Position.Left} style={{ background: "#60a5fa" }} />
      <Handle type="source" position={Position.Right} style={{ background: "#34d399" }} />
      {visibilityState !== "internal" && (
        <div style={{ fontSize: 9, color: isExternal ? "#818cf8" : isFiltered ? "#38bdf8" : "#fbbf24", marginBottom: 1 }}>
          {isExternal ? "external" : isFiltered ? "filtered peer" : "semantic peer"}
        </div>
      )}
      <div style={{ fontWeight: 600, color: isExternal ? "#a5b4fc" : isFiltered ? "#7dd3fc" : isSemantic ? "#fcd34d" : "#e2e8f0" }}>{topic}</div>
      <div style={{ color: "#64748b", fontSize: 10 }}>{msgType}</div>
    </div>
  );
}

// ── External action / service endpoint bridge ────────────────────────────────
function ExternalEndpointCard({ data }: NodeProps) {
  const { name, endpointType, role, presence } = data as {
    name: string;
    endpointType: "action" | "service";
    role: "server" | "client";  // the MISSING side
    presence: EndpointPresence;
  };
  const isAction = endpointType === "action";
  const color = isAction ? "#fcd34d" : "#d8b4fe";
  const bg = presence === "semantic"
    ? (isAction ? "#241906" : "#211432")
    : presence === "filtered"
      ? (isAction ? "#1c1605" : "#15122e")
      : (isAction ? "#1c1500" : "#1e1028");
  return (
    <div style={{
      border: `1px dashed ${color}`,
      borderRadius: 8, background: bg,
      padding: "5px 10px", fontFamily: "monospace", fontSize: 11,
      minWidth: 140, textAlign: "center", color: "#94a3b8",
    }}>
      <Handle type="target" position={Position.Left} style={{ background: color }} />
      <Handle type="source" position={Position.Right} style={{ background: color }} />
      <div style={{ fontSize: 9, color, marginBottom: 2, display: "flex", alignItems: "center", justifyContent: "center", gap: 3 }}>
        {isAction ? <Zap size={9} /> : <Wrench size={9} />}
        {endpointType} {role} ({presence === "semantic" ? "semantic" : presence === "filtered" ? "filtered" : "external"})
      </div>
      <div style={{ fontWeight: 600, color: "#e2e8f0", wordBreak: "break-all" }}>{name}</div>
    </div>
  );
}

function SemanticEndpointCard({ data }: NodeProps) {
  const { endpointType, labels } = data as {
    endpointType: "service" | "action";
    labels: string[];
  };
  const isAction = endpointType === "action";
  const color = isAction ? "#fcd34d" : "#f59e0b";
  const title = labels.length <= 3 ? labels.join(" · ") : `${labels.length} semantic ${isAction ? "actions" : "services"}`;

  return (
    <div style={{
      border: `1px dashed ${color}`,
      borderRadius: 8,
      background: isAction ? "#241906" : "#211406",
      padding: "5px 10px",
      fontFamily: "monospace",
      fontSize: 11,
      minWidth: 160,
      textAlign: "center",
      color: "#e2e8f0",
    }}>
      <Handle type="target" position={Position.Left} style={{ background: color }} />
      <Handle type="source" position={Position.Right} style={{ background: color }} />
      <div style={{ fontSize: 9, color, marginBottom: 2 }}>
        semantic {endpointType}
      </div>
      <div style={{ fontWeight: 600, wordBreak: "break-word" }}>{title}</div>
    </div>
  );
}

const nodeTypes = {
  rosNode: RosNodeCard,
  topicBridge: TopicBridgeCard,
  externalEndpoint: ExternalEndpointCard,
  semanticEndpoint: SemanticEndpointCard,
};

// ── Dagre auto-layout (left-to-right hierarchical) ───────────────────────────
const NODE_W = 240;
const NODE_H = 100;
const BRIDGE_W = 180;
const BRIDGE_H = 54;

function getLayoutSize(nodeType: string | undefined): { width: number; height: number } {
  if (nodeType === "rosNode") return { width: NODE_W, height: NODE_H };
  return { width: BRIDGE_W, height: BRIDGE_H };
}

function applyDagreLayout(nodes: Node[], edges: Edge[]): Node[] {
  if (nodes.length === 0) return nodes;
  const g = new Dagre.graphlib.Graph().setDefaultEdgeLabel(() => ({}));
  g.setGraph({ rankdir: "LR", nodesep: 60, ranksep: 120, edgesep: 20 });
  for (const n of nodes) {
    const { width, height } = getLayoutSize(n.type);
    g.setNode(n.id, { width, height });
  }
  for (const e of edges) {
    if (g.hasNode(e.source) && g.hasNode(e.target)) {
      g.setEdge(e.source, e.target);
    }
  }
  Dagre.layout(g);
  return nodes.map((n) => {
    const pos = g.node(n.id);
    const { width, height } = getLayoutSize(n.type);
    return { ...n, position: { x: pos.x - width / 2, y: pos.y - height / 2 } };
  });
}

// ── Compute connected IDs from edges (1 hop) ─────────────────────────────────
function getConnectedIdsFromEdges(selectedId: string, edges: Edge[]): Set<string> {
  const connected = new Set<string>([selectedId]);
  for (const e of edges) {
    if (e.source === selectedId) connected.add(e.target);
    if (e.target === selectedId) connected.add(e.source);
  }
  return connected;
}

// ── Helpers for message-type lookups ────────────────────────────────────────
const shortType = (t: string) => t.split("/").pop() ?? t;
function findMsgDef(messages: MsgDef[], typeStr: string): MsgDef | undefined {
  const s = shortType(typeStr);
  return messages.find((m) => m.name === s || `${m.package}/${m.name}` === typeStr);
}

function uniqueStrings(values: string[]): string[] {
  return [...new Set(values)];
}

function normalizeRosName(name: string): string {
  return name.replace(/^\/+/, "").replace(/^~\/?/, "").trim();
}

function semanticEndpointKey(name: string): string {
  const normalized = normalizeRosName(name);
  if (!normalized) return "";
  const parts = normalized.split("/").filter(Boolean);
  if (parts.length <= 1) return normalized;
  return parts.slice(-2).join("/");
}

function semanticEndpointAliases(name: string): string[] {
  const normalized = normalizeRosName(name);
  if (!normalized) return [];
  const parts = normalized.split("/").filter(Boolean);
  return parts.map((_, index) => parts.slice(index).join("/"));
}

function findSharedSemanticEndpoint(left: string, right: string): string {
  const rightAliases = new Set(semanticEndpointAliases(right));
  return semanticEndpointAliases(left).find((alias) => rightAliases.has(alias)) ?? "";
}

function sameSemanticEndpointName(left: string, right: string): boolean {
  return findSharedSemanticEndpoint(left, right) !== "";
}

function sameSemanticType(left: string, right: string): boolean {
  return shortType(left) === shortType(right);
}

function getPerformerHelperFamily(node: RosNode): string | null {
  if (node.package !== "performer_helper") return null;
  const match = node.nodeName.match(/^(.*)_performer_helper$/);
  return match ? match[1] : null;
}

function getPrimaryAdapterFamily(node: RosNode): string | null {
  if (!node.package.endsWith("_adapter")) return null;
  const family = node.package.slice(0, -"_adapter".length);
  return node.nodeName === `${family}_adapter` ? family : null;
}

function isEmbeddedHelperNode(node: RosNode): boolean {
  return getPerformerHelperFamily(node) !== null;
}

function isPerformerHelperOwnerNode(node: RosNode): boolean {
  return node.package === "performer_helper" && node.nodeName === "performer_helper";
}

function areSemanticPeers(left: RosNode, right: RosNode): boolean {
  const leftHelperFamily = getPerformerHelperFamily(left);
  const rightHelperFamily = getPerformerHelperFamily(right);
  const leftAdapterFamily = getPrimaryAdapterFamily(left);
  const rightAdapterFamily = getPrimaryAdapterFamily(right);
  return (leftHelperFamily !== null && leftHelperFamily === rightAdapterFamily)
    || (rightHelperFamily !== null && rightHelperFamily === leftAdapterFamily);
}

function findSemanticServicePeers(
  node: RosNode,
  serviceName: string,
  srvType: string,
  role: "server" | "client",
  scopeNodes: RosNode[],
): RosNode[] {
  return scopeNodes.filter((peer) => (
    peer.id !== node.id
    && peer.services.some((service) => (
      service.role === role
      && service.service !== serviceName
      && sameSemanticEndpointName(service.service, serviceName)
      && sameSemanticType(service.srvType, srvType)
    ))
  ));
}

function findSemanticActionPeers(
  node: RosNode,
  actionName: string,
  actionType: string,
  role: "server" | "client",
  scopeNodes: RosNode[],
): RosNode[] {
  return scopeNodes.filter((peer) => (
    peer.id !== node.id
    && peer.actions.some((action) => (
      action.role === role
      && action.action !== actionName
      && sameSemanticEndpointName(action.action, actionName)
      && sameSemanticType(action.actionType, actionType)
    ))
  ));
}

function findSemanticTopicPeers(
  node: RosNode,
  topicName: string,
  msgType: string,
  direction: "pub" | "sub",
  scopeNodes: RosNode[],
): RosNode[] {
  return scopeNodes.filter((peer) => (
    peer.id !== node.id
    && peer.topics.some((topic) => (
      topic.direction === direction
      && topic.topic !== topicName
      && sameSemanticEndpointName(topic.topic, topicName)
      && sameSemanticType(topic.msgType, msgType)
    ))
  ));
}

function collectSemanticAffinity(helperNode: RosNode, adapterNode: RosNode): SemanticAffinity {
  if (!areSemanticPeers(helperNode, adapterNode)) {
    return { services: [], actions: [], topics: [] };
  }

  const services = uniqueStrings(
    helperNode.services
      .filter((service) => service.role === "client")
      .flatMap((service) => adapterNode.services
        .filter((peerService) => (
          peerService.role === "server"
          && peerService.service !== service.service
          && sameSemanticEndpointName(peerService.service, service.service)
          && sameSemanticType(peerService.srvType, service.srvType)
        ))
        .map((peerService) => findSharedSemanticEndpoint(peerService.service, service.service) || semanticEndpointKey(service.service))),
  );

  const actions = uniqueStrings(
    helperNode.actions
      .filter((action) => action.role === "client")
      .flatMap((action) => adapterNode.actions
        .filter((peerAction) => (
          peerAction.role === "server"
          && peerAction.action !== action.action
          && sameSemanticEndpointName(peerAction.action, action.action)
          && sameSemanticType(peerAction.actionType, action.actionType)
        ))
        .map((peerAction) => findSharedSemanticEndpoint(peerAction.action, action.action) || semanticEndpointKey(action.action))),
  );

  const topics = uniqueStrings(
    helperNode.topics
      .filter((topic) => topic.direction === "sub")
      .flatMap((topic) => adapterNode.topics
        .filter((peerTopic) => (
          peerTopic.direction === "pub"
          && peerTopic.topic !== topic.topic
          && sameSemanticEndpointName(peerTopic.topic, topic.topic)
          && sameSemanticType(peerTopic.msgType, topic.msgType)
        ))
        .map((peerTopic) => findSharedSemanticEndpoint(peerTopic.topic, topic.topic) || semanticEndpointKey(topic.topic))),
  );

  return { services, actions, topics };
}

function semanticEdgeLabel(affinity: SemanticAffinity): string {
  const parts = [
    affinity.services.length > 0 ? `${affinity.services.length} srv` : null,
    affinity.actions.length > 0 ? `${affinity.actions.length} act` : null,
    affinity.topics.length > 0 ? `${affinity.topics.length} topic` : null,
  ].filter(Boolean);
  return parts.join(" · ");
}

// ── Main view ─────────────────────────────────────────────────────────────────
export function NodeGraphView() {
  const {
    data,
    showTopics,
    showServices,
    showActions,
    toolIntegrations,
    selectedPackages,
    selectedNodeIds,
    topicFilters,
    msgTypeFilters,
  } = useStore();

  const [selectedItemId, setSelectedItemId] = useState<string | null>(null);
  const [selectedEdge, setSelectedEdge] = useState<Edge | null>(null);

  const allNodes = useMemo(() => {
    if (!data) return [] as RosNode[];
    return data.nodes.concat(buildKnownToolNodes(data, toolIntegrations));
  }, [data, toolIntegrations]);

  const handleNodeClick = useCallback((_evt: React.MouseEvent, node: Node) => {
    setSelectedEdge(null);
    setSelectedItemId((prev) => (prev === node.id ? null : node.id));
  }, []);

  const handleEdgeClick = useCallback((_evt: React.MouseEvent, edge: Edge) => {
    setSelectedItemId(null);
    setSelectedEdge((prev) => (prev?.id === edge.id ? null : edge));
  }, []);

  const handlePaneClick = useCallback(() => {
    setSelectedItemId(null);
    setSelectedEdge(null);
  }, []);

  const { nodes: flowNodes, edges: flowEdges } = useMemo(() => {
    if (!data) return { nodes: [], edges: [] };

    const rosNodes = allNodes.filter((n) => n.sourceKind === "knownTool"
      ? true
      : selectedPackages.has(n.package) && selectedNodeIds.has(n.id));
    const rosNodeById = new Map(rosNodes.map((node) => [node.id, node]));
    const allTopicPublishers = new Map<string, string[]>();
    const allTopicSubscribers = new Map<string, string[]>();
    const allServiceServers = new Map<string, { nodeId: string; srvType: string }[]>();
    const allServiceClients = new Map<string, string[]>();
    const allActionServers = new Map<string, { nodeId: string; actionType: string }[]>();
    const allActionClients = new Map<string, string[]>();

    for (const node of allNodes) {
      for (const topic of node.topics) {
        if (topic.direction === "pub") {
          const publishers = allTopicPublishers.get(topic.topic) ?? [];
          publishers.push(node.id);
          allTopicPublishers.set(topic.topic, publishers);
        } else {
          const subscribers = allTopicSubscribers.get(topic.topic) ?? [];
          subscribers.push(node.id);
          allTopicSubscribers.set(topic.topic, subscribers);
        }
      }
      for (const service of node.services) {
        if (service.role === "server") {
          const servers = allServiceServers.get(service.service) ?? [];
          servers.push({ nodeId: node.id, srvType: service.srvType });
          allServiceServers.set(service.service, servers);
        } else {
          const clients = allServiceClients.get(service.service) ?? [];
          clients.push(node.id);
          allServiceClients.set(service.service, clients);
        }
      }
      for (const action of node.actions) {
        if (action.role === "server") {
          const servers = allActionServers.get(action.action) ?? [];
          servers.push({ nodeId: node.id, actionType: action.actionType });
          allActionServers.set(action.action, servers);
        } else {
          const clients = allActionClients.get(action.action) ?? [];
          clients.push(node.id);
          allActionClients.set(action.action, clients);
        }
      }
    }

    const rfNodes: Node[] = rosNodes.map((n) => ({
      id: n.id,
      type: "rosNode",
      position: { x: 0, y: 0 },
      data: { node: n },
    }));

    const rfEdges: Edge[] = [];
    const semanticBridgeMeta = new Map<string, { nodeIndex: number; labels: Set<string> }>();
    const topicBridgeMap = new Map<string, string>(); // topic → bridge node id
    const topicBridgeMetadata = new Map<string, { topic: string; msgType: string }>();

    const connectionPass = (name: string) => topicFilters.size === 0
      || [...topicFilters].some((filterValue) => sameSemanticEndpointName(filterValue, name));
    const typePass = (t: string) => msgTypeFilters.size === 0 || msgTypeFilters.has(t) || [...msgTypeFilters].some((f) => shortType(f) === shortType(t));
    const pushSemanticEdge = ({
      kind,
      source,
      target,
      label,
      color,
      dasharray,
      width,
    }: {
      kind: "service" | "action";
      source: string;
      target: string;
      label: string;
      color: string;
      dasharray: string;
      width: number;
    }) => {
      const pairKey = `${kind}:${source}:${target}`;
      const existing = semanticBridgeMeta.get(pairKey);
      if (existing) {
        existing.labels.add(label);
        rfNodes[existing.nodeIndex] = {
          ...rfNodes[existing.nodeIndex],
          data: {
            ...(rfNodes[existing.nodeIndex].data as object),
            labels: [...existing.labels],
          },
        };
        return;
      }

      const labels = new Set([label]);
      const bridgeId = `sem_bridge__${kind}__${source}__${target}`;
      const nodeIndex = rfNodes.push({
        id: bridgeId,
        type: "semanticEndpoint",
        position: { x: 0, y: 0 },
        data: {
          endpointType: kind,
          labels: [...labels],
          sourceId: source,
          targetId: target,
        },
      }) - 1;
      rfEdges.push({
        id: `${bridgeId}__in`,
        source,
        target: bridgeId,
        style: { stroke: color, strokeDasharray: dasharray, strokeWidth: width },
        markerEnd: { type: MarkerType.ArrowClosed, color },
      });
      rfEdges.push({
        id: `${bridgeId}__out`,
        source: bridgeId,
        target,
        style: { stroke: color, strokeDasharray: dasharray, strokeWidth: width },
        markerEnd: { type: MarkerType.ArrowClosed, color },
      });
      semanticBridgeMeta.set(pairKey, { nodeIndex, labels });
    };
    const findCompatibleTopicBridge = (topic: string, msgType: string): string | null => {
      for (const [bridgeId, metadata] of topicBridgeMetadata) {
        if (!sameSemanticType(metadata.msgType, msgType)) continue;
        if (!sameSemanticEndpointName(metadata.topic, topic)) continue;
        return bridgeId;
      }
      return null;
    };
    const exactTopicVisibilityState = (topic: string): TopicBridgeVisibility => {
      const visiblePublishers = rosNodes.filter((node) => node.topics.some((item) => item.direction === "pub" && item.topic === topic)).length;
      const visibleSubscribers = rosNodes.filter((node) => node.topics.some((item) => item.direction === "sub" && item.topic === topic)).length;
      const fullPublishers = (allTopicPublishers.get(topic) ?? []).length;
      const fullSubscribers = (allTopicSubscribers.get(topic) ?? []).length;

      if (visiblePublishers === 0 && fullPublishers === 0) return "external";
      if ((visiblePublishers === 0 && fullPublishers > 0) || (visibleSubscribers === 0 && fullSubscribers > 0)) {
        return "filtered";
      }
      return "internal";
    };

    if (showTopics) {
      // Pass 1: create bridge nodes for all published topics
      for (const n of rosNodes) {
        for (const t of n.topics) {
          if (t.direction !== "pub") continue;
          if (!connectionPass(t.topic) || !typePass(t.msgType)) continue;
          if (!topicBridgeMap.has(t.topic)) {
            const compatibleBridgeId = findCompatibleTopicBridge(t.topic, t.msgType);
            if (compatibleBridgeId) {
              topicBridgeMap.set(t.topic, compatibleBridgeId);
            } else {
              const visibleSemanticSubscribers = findSemanticTopicPeers(n, t.topic, t.msgType, "sub", rosNodes);
              const allSemanticSubscribers = findSemanticTopicPeers(n, t.topic, t.msgType, "sub", allNodes);
              const hasExactVisibleSubscribers = rosNodes.some((node) => node.topics.some((item) => item.direction === "sub" && item.topic === t.topic));
              const visibilityState: TopicBridgeVisibility = hasExactVisibleSubscribers
                ? "internal"
                : visibleSemanticSubscribers.length > 0
                  ? "semantic"
                  : ((allTopicSubscribers.get(t.topic) ?? []).length > 0 || allSemanticSubscribers.length > 0)
                    ? "filtered"
                    : exactTopicVisibilityState(t.topic);
              const bridgeId = `topic__${t.topic.replace(/\//g, "_")}`;
              topicBridgeMap.set(t.topic, bridgeId);
              topicBridgeMetadata.set(bridgeId, { topic: t.topic, msgType: t.msgType });
              rfNodes.push({
                id: bridgeId,
                type: "topicBridge",
                position: { x: 0, y: 0 },
                data: {
                  topic: t.topic,
                  msgType: t.msgType,
                  visibilityState,
                  semanticPeerIds: visibleSemanticSubscribers.map((peer) => peer.id),
                },
              });
            }
          }
          rfEdges.push({
            id: `${n.id}--pub--${t.topic}`,
            source: n.id,
            target: topicBridgeMap.get(t.topic)!,
            markerEnd: { type: MarkerType.ArrowClosed, color: "#67e8f9" },
            style: { stroke: "#67e8f9", strokeWidth: 1.5 },
          });
        }
      }
      // Pass 2: create "external source" bridges for subscribed-only topics
      for (const n of rosNodes) {
        for (const t of n.topics) {
          if (t.direction !== "sub") continue;
          if (!connectionPass(t.topic) || !typePass(t.msgType)) continue;
          if (topicBridgeMap.has(t.topic)) continue; // publisher bridge already exists
          const compatibleBridgeId = findCompatibleTopicBridge(t.topic, t.msgType);
          if (compatibleBridgeId) {
            topicBridgeMap.set(t.topic, compatibleBridgeId);
            continue;
          }
          const visibleSemanticPublishers = findSemanticTopicPeers(n, t.topic, t.msgType, "pub", rosNodes);
          const allSemanticPublishers = findSemanticTopicPeers(n, t.topic, t.msgType, "pub", allNodes);
          const visibilityState: TopicBridgeVisibility = visibleSemanticPublishers.length > 0
            ? "semantic"
            : ((allTopicPublishers.get(t.topic) ?? []).length > 0 || allSemanticPublishers.length > 0)
              ? "filtered"
              : exactTopicVisibilityState(t.topic);
          const bridgeId = `topic__${t.topic.replace(/\//g, "_")}`;
          topicBridgeMap.set(t.topic, bridgeId);
          topicBridgeMetadata.set(bridgeId, { topic: t.topic, msgType: t.msgType });
          rfNodes.push({
            id: bridgeId,
            type: "topicBridge",
            position: { x: 0, y: 0 },
            data: {
              topic: t.topic,
              msgType: t.msgType,
              visibilityState,
              semanticPeerIds: visibleSemanticPublishers.map((peer) => peer.id),
            },
          });
        }
      }
      // Pass 3: connect all subscribers to their bridge (internal or external)
      for (const n of rosNodes) {
        for (const t of n.topics) {
          if (t.direction !== "sub") continue;
          if (!connectionPass(t.topic) || !typePass(t.msgType)) continue;
          const bridgeId = topicBridgeMap.get(t.topic);
          if (!bridgeId) continue;
          rfEdges.push({
            id: `${n.id}--sub--${t.topic}`,
            source: bridgeId,
            target: n.id,
            markerEnd: { type: MarkerType.ArrowClosed, color: "#86efac" },
            style: { stroke: "#86efac", strokeWidth: 1.5 },
          });
        }
      }
    }

    if (showServices) {
      // Build server/client maps for services
      const svcServers = new Map<string, { nodeId: string; srvType: string }>();
      const svcClients = new Map<string, string[]>(); // service → node.id[]
      for (const n of rosNodes) {
        for (const svc of n.services) {
          if (!typePass(svc.srvType)) continue;
          if (!connectionPass(svc.service)) continue;
          if (svc.role === "server") {
            if (!svcServers.has(svc.service)) svcServers.set(svc.service, { nodeId: n.id, srvType: svc.srvType });
          } else {
            const arr = svcClients.get(svc.service) ?? [];
            arr.push(n.id);
            svcClients.set(svc.service, arr);
          }
        }
      }
      // Matched: client → server
      for (const [svcName, { nodeId: serverId, srvType }] of svcServers) {
        for (const clientId of svcClients.get(svcName) ?? []) {
          rfEdges.push({
            id: `${clientId}--svc--${svcName}`,
            source: clientId, target: serverId,
            label: svcName,
            labelStyle: { fill: "#d8b4fe", fontSize: 10 },
            style: { stroke: "#d8b4fe", strokeDasharray: "5,3", strokeWidth: 1.5 },
            markerEnd: { type: MarkerType.ArrowClosed, color: "#d8b4fe" },
            data: { kind: "service", service: svcName, srvType, clientId, serverId },
          });
        }
      }
      // Unmatched server → external client bridge
      for (const [svcName, { nodeId: serverId, srvType }] of svcServers) {
        if ((svcClients.get(svcName) ?? []).length === 0) {
          const serverNode = rosNodeById.get(serverId);
          const allClients = allServiceClients.get(svcName) ?? [];
          const visibleSemanticClients = serverNode
            ? findSemanticServicePeers(serverNode, svcName, srvType, "client", rosNodes)
            : [];
          const allSemanticClients = serverNode
            ? findSemanticServicePeers(serverNode, svcName, srvType, "client", allNodes)
            : [];
          if (visibleSemanticClients.length > 0) {
            for (const clientNode of visibleSemanticClients) {
              const clientService = clientNode.services.find((service) => (
                service.role === "client"
                && service.service !== svcName
                && sameSemanticEndpointName(service.service, svcName)
                && sameSemanticType(service.srvType, srvType)
              ));
              const semanticName = clientService
                ? findSharedSemanticEndpoint(clientService.service, svcName) || svcName
                : svcName;
              pushSemanticEdge({
                kind: "service",
                source: clientNode.id,
                target: serverId,
                label: semanticName,
                color: "#f59e0b",
                dasharray: "3,5",
                width: 1.5,
              });
            }
            continue;
          }
          const presence: EndpointPresence = visibleSemanticClients.length > 0
            ? "semantic"
            : (allClients.length > 0 || allSemanticClients.length > 0)
              ? "filtered"
              : "external";
          if (presence !== "external") continue;
          const bridgeId = `svc_ext_c__${svcName.replace(/\//g, "_")}`;
          rfNodes.push({ id: bridgeId, type: "externalEndpoint", position: { x: 0, y: 0 },
            data: {
              name: svcName,
              endpointType: "service",
              role: "client",
              presence,
              semanticPeerIds: visibleSemanticClients.map((peer) => peer.id),
            } });
          rfEdges.push({ id: `${serverId}--svc_ext--${svcName}`, source: serverId, target: bridgeId,
            style: { stroke: "#7c5caa", strokeDasharray: "3,4", strokeWidth: 1 },
            markerEnd: { type: MarkerType.ArrowClosed, color: "#7c5caa" } });
        }
      }
      // Unmatched client → external server bridge
      for (const [svcName, clientIds] of svcClients) {
        if (!svcServers.has(svcName)) {
          const allServers = allServiceServers.get(svcName) ?? [];
          const visibleSemanticServers = uniqueStrings(clientIds.flatMap((clientId) => {
            const clientNode = rosNodeById.get(clientId);
            const clientService = clientNode?.services.find((service) => service.role === "client" && service.service === svcName);
            if (!clientNode || !clientService) return [];
            return findSemanticServicePeers(clientNode, svcName, clientService.srvType, "server", rosNodes).map((peer) => peer.id);
          }));
          const allSemanticServers = uniqueStrings(clientIds.flatMap((clientId) => {
            const clientNode = rosNodeById.get(clientId);
            const clientService = clientNode?.services.find((service) => service.role === "client" && service.service === svcName);
            if (!clientNode || !clientService) return [];
            return findSemanticServicePeers(clientNode, svcName, clientService.srvType, "server", allNodes).map((peer) => peer.id);
          }));
          if (visibleSemanticServers.length > 0) {
            for (const clientId of clientIds) {
              const clientNode = rosNodeById.get(clientId);
              const clientService = clientNode?.services.find((service) => service.role === "client" && service.service === svcName);
              if (!clientNode || !clientService) continue;
              for (const semanticServerId of visibleSemanticServers) {
                const semanticServerNode = rosNodeById.get(semanticServerId);
                const semanticServer = semanticServerNode?.services.find((service) => (
                  service.role === "server"
                  && service.service !== svcName
                  && sameSemanticEndpointName(service.service, svcName)
                  && sameSemanticType(service.srvType, clientService.srvType)
                ));
                const semanticName = semanticServer
                  ? findSharedSemanticEndpoint(semanticServer.service, svcName) || svcName
                  : svcName;
                if (!semanticServerNode) continue;
                pushSemanticEdge({
                  kind: "service",
                  source: clientId,
                  target: semanticServerId,
                  label: semanticName,
                  color: "#f59e0b",
                  dasharray: "3,5",
                  width: 1.5,
                });
              }
            }
            continue;
          }
          const presence: EndpointPresence = visibleSemanticServers.length > 0
            ? "semantic"
            : (allServers.length > 0 || allSemanticServers.length > 0)
              ? "filtered"
              : "external";
          if (presence !== "external") continue;
          const bridgeId = `svc_ext_s__${svcName.replace(/\//g, "_")}`;
          rfNodes.push({ id: bridgeId, type: "externalEndpoint", position: { x: 0, y: 0 },
            data: {
              name: svcName,
              endpointType: "service",
              role: "server",
              presence,
              semanticPeerIds: visibleSemanticServers,
            } });
          for (const clientId of clientIds) {
            rfEdges.push({ id: `${clientId}--svc_ext--${svcName}`, source: clientId, target: bridgeId,
              style: { stroke: "#7c5caa", strokeDasharray: "3,4", strokeWidth: 1 },
              markerEnd: { type: MarkerType.ArrowClosed, color: "#7c5caa" } });
          }
        }
      }
    }

    if (showActions) {
      // Build server/client maps for actions
      const actServers = new Map<string, { nodeId: string; actionType: string }>();
      const actClients = new Map<string, string[]>(); // action → node.id[]
      for (const n of rosNodes) {
        for (const act of n.actions) {
          if (!typePass(act.actionType)) continue;
          if (!connectionPass(act.action)) continue;
          if (act.role === "server") {
            if (!actServers.has(act.action)) actServers.set(act.action, { nodeId: n.id, actionType: act.actionType });
          } else {
            const arr = actClients.get(act.action) ?? [];
            arr.push(n.id);
            actClients.set(act.action, arr);
          }
        }
      }
      // Matched: client → server
      for (const [actName, { nodeId: serverId, actionType }] of actServers) {
        for (const clientId of actClients.get(actName) ?? []) {
          rfEdges.push({
            id: `${clientId}--act--${actName}`,
            source: clientId, target: serverId,
            label: actName,
            labelStyle: { fill: "#fcd34d", fontSize: 10 },
            style: { stroke: "#fcd34d", strokeDasharray: "8,3", strokeWidth: 2 },
            markerEnd: { type: MarkerType.ArrowClosed, color: "#fcd34d" },
            data: { kind: "action", action: actName, actionType, clientId, serverId },
          });
        }
      }
      // Unmatched server → external client bridge (e.g. follow_joint_trajectory waiting for MoveIt)
      for (const [actName, { nodeId: serverId, actionType }] of actServers) {
        if ((actClients.get(actName) ?? []).length === 0) {
          const serverNode = rosNodeById.get(serverId);
          const allClients = allActionClients.get(actName) ?? [];
          const visibleSemanticClients = serverNode
            ? findSemanticActionPeers(serverNode, actName, actionType, "client", rosNodes)
            : [];
          const allSemanticClients = serverNode
            ? findSemanticActionPeers(serverNode, actName, actionType, "client", allNodes)
            : [];
          if (visibleSemanticClients.length > 0) {
            for (const clientNode of visibleSemanticClients) {
              const clientAction = clientNode.actions.find((action) => (
                action.role === "client"
                && action.action !== actName
                && sameSemanticEndpointName(action.action, actName)
                && sameSemanticType(action.actionType, actionType)
              ));
              const semanticName = clientAction
                ? findSharedSemanticEndpoint(clientAction.action, actName) || actName
                : actName;
              pushSemanticEdge({
                kind: "action",
                source: clientNode.id,
                target: serverId,
                label: semanticName,
                color: "#f59e0b",
                dasharray: "4,4",
                width: 1.6,
              });
            }
            continue;
          }
          const presence: EndpointPresence = visibleSemanticClients.length > 0
            ? "semantic"
            : (allClients.length > 0 || allSemanticClients.length > 0)
              ? "filtered"
              : "external";
          if (presence !== "external") continue;
          const bridgeId = `act_ext_c__${actName.replace(/\//g, "_")}`;
          rfNodes.push({ id: bridgeId, type: "externalEndpoint", position: { x: 0, y: 0 },
            data: {
              name: actName,
              endpointType: "action",
              role: "client",
              presence,
              semanticPeerIds: visibleSemanticClients.map((peer) => peer.id),
            } });
          rfEdges.push({ id: `${serverId}--act_ext--${actName}`, source: serverId, target: bridgeId,
            style: { stroke: "#92700e", strokeDasharray: "4,4", strokeWidth: 1.5 },
            markerEnd: { type: MarkerType.ArrowClosed, color: "#92700e" } });
        }
      }
      // Unmatched client → external server bridge
      for (const [actName, clientIds] of actClients) {
        if (!actServers.has(actName)) {
          const allServers = allActionServers.get(actName) ?? [];
          const visibleSemanticServers = uniqueStrings(clientIds.flatMap((clientId) => {
            const clientNode = rosNodeById.get(clientId);
            const clientAction = clientNode?.actions.find((action) => action.role === "client" && action.action === actName);
            if (!clientNode || !clientAction) return [];
            return findSemanticActionPeers(clientNode, actName, clientAction.actionType, "server", rosNodes).map((peer) => peer.id);
          }));
          const allSemanticServers = uniqueStrings(clientIds.flatMap((clientId) => {
            const clientNode = rosNodeById.get(clientId);
            const clientAction = clientNode?.actions.find((action) => action.role === "client" && action.action === actName);
            if (!clientNode || !clientAction) return [];
            return findSemanticActionPeers(clientNode, actName, clientAction.actionType, "server", allNodes).map((peer) => peer.id);
          }));
          if (visibleSemanticServers.length > 0) {
            for (const clientId of clientIds) {
              const clientNode = rosNodeById.get(clientId);
              const clientAction = clientNode?.actions.find((action) => action.role === "client" && action.action === actName);
              if (!clientNode || !clientAction) continue;
              for (const semanticServerId of visibleSemanticServers) {
                const semanticServerNode = rosNodeById.get(semanticServerId);
                const semanticServer = semanticServerNode?.actions.find((action) => (
                  action.role === "server"
                  && action.action !== actName
                  && sameSemanticEndpointName(action.action, actName)
                  && sameSemanticType(action.actionType, clientAction.actionType)
                ));
                const semanticName = semanticServer
                  ? findSharedSemanticEndpoint(semanticServer.action, actName) || actName
                  : actName;
                if (!semanticServerNode) continue;
                pushSemanticEdge({
                  kind: "action",
                  source: clientId,
                  target: semanticServerId,
                  label: semanticName,
                  color: "#f59e0b",
                  dasharray: "4,4",
                  width: 1.6,
                });
              }
            }
            continue;
          }
          const presence: EndpointPresence = visibleSemanticServers.length > 0
            ? "semantic"
            : (allServers.length > 0 || allSemanticServers.length > 0)
              ? "filtered"
              : "external";
          if (presence !== "external") continue;
          const bridgeId = `act_ext_s__${actName.replace(/\//g, "_")}`;
          rfNodes.push({ id: bridgeId, type: "externalEndpoint", position: { x: 0, y: 0 },
            data: {
              name: actName,
              endpointType: "action",
              role: "server",
              presence,
              semanticPeerIds: visibleSemanticServers,
            } });
          for (const clientId of clientIds) {
            rfEdges.push({ id: `${clientId}--act_ext--${actName}`, source: clientId, target: bridgeId,
              style: { stroke: "#92700e", strokeDasharray: "4,4", strokeWidth: 1.5 },
              markerEnd: { type: MarkerType.ArrowClosed, color: "#92700e" } });
          }
        }
      }
    }

    const performerHelperOwner = rosNodes.find(isPerformerHelperOwnerNode);
    if (performerHelperOwner) {
      for (const helperNode of rosNodes.filter(isEmbeddedHelperNode)) {
        rfEdges.push({
          id: `${performerHelperOwner.id}--semantic-owner--${helperNode.id}`,
          source: performerHelperOwner.id,
          target: helperNode.id,
          label: "embedded helper",
          labelStyle: { fill: "#cbd5e1", fontSize: 10 },
          style: { stroke: "#94a3b8", strokeDasharray: "2,6", strokeWidth: 1.2 },
          markerEnd: { type: MarkerType.ArrowClosed, color: "#94a3b8" },
        });
      }
    }

    for (const helperNode of rosNodes.filter(isEmbeddedHelperNode)) {
      const family = getPerformerHelperFamily(helperNode);
      const adapterNode = rosNodes.find((node) => getPrimaryAdapterFamily(node) === family);
      if (!family || !adapterNode) continue;
      const affinity = collectSemanticAffinity(helperNode, adapterNode);
      if (affinity.services.length === 0 && affinity.actions.length === 0 && affinity.topics.length === 0) continue;
      rfEdges.push({
        id: `${helperNode.id}--semantic-affinity--${adapterNode.id}`,
        source: helperNode.id,
        target: adapterNode.id,
        label: semanticEdgeLabel(affinity),
        labelStyle: { fill: "#fbbf24", fontSize: 10 },
        style: { stroke: "#f59e0b", strokeDasharray: "7,5", strokeWidth: 1.4 },
        markerEnd: { type: MarkerType.ArrowClosed, color: "#f59e0b" },
      });
    }

    // ── Apply selection-based dimming ────────────────────────────────────────────
    const visibleNodeIds = new Set(rfNodes.map((node) => node.id));
    const activeSelectedItemId = selectedItemId && visibleNodeIds.has(selectedItemId)
      ? selectedItemId
      : null;
    const activeSelectedEdge = selectedEdge && rfEdges.some((edge) => edge.id === selectedEdge.id)
      ? selectedEdge
      : null;

    const connectedIds = activeSelectedItemId
      ? getConnectedIdsFromEdges(activeSelectedItemId, rfEdges)
      : activeSelectedEdge
        ? new Set([activeSelectedEdge.source, activeSelectedEdge.target])
        : null;

    const finalNodes = rfNodes.map((n) => ({
      ...n,
      data: { ...(n.data as object), isSelected: n.id === activeSelectedItemId },
      style: {
        opacity: connectedIds ? (connectedIds.has(n.id) ? 1 : 0.15) : 1,
        transition: "opacity 0.2s",
      },
    }));

    const finalEdges = rfEdges.map((e) => ({
      ...e,
      style: {
        ...e.style,
        opacity: connectedIds
          ? (activeSelectedEdge
              ? (e.id === activeSelectedEdge.id ? 1 : 0.06)
              : (connectedIds.has(e.source) && connectedIds.has(e.target) ? 1 : 0.06))
          : 1,
        transition: "opacity 0.2s",
      },
    }));

    return { nodes: applyDagreLayout(finalNodes, finalEdges), edges: finalEdges };
  }, [allNodes, data, showTopics, showServices, showActions, selectedPackages, selectedNodeIds, topicFilters, msgTypeFilters, selectedItemId, selectedEdge]);

  useEffect(() => {
    if (selectedItemId && !flowNodes.some((node) => node.id === selectedItemId)) {
      setSelectedItemId(null);
    }
  }, [flowNodes, selectedItemId]);

  useEffect(() => {
    if (selectedEdge && !flowEdges.some((edge) => edge.id === selectedEdge.id)) {
      setSelectedEdge(null);
    }
  }, [flowEdges, selectedEdge]);

  if (!data) return <div style={{ padding: 40, color: "#94a3b8" }}>No workspace data loaded.</div>;

  const clearSelection = () => { setSelectedItemId(null); setSelectedEdge(null); };
  const selectedRosNode = selectedItemId ? allNodes.find((n) => n.id === selectedItemId) ?? null : null;
  const selectedFlowNode = selectedItemId ? flowNodes.find((n) => n.id === selectedItemId) ?? null : null;

  return (
    <div style={{ display: "flex", height: "100%", overflow: "hidden" }}>
      <div style={{ flex: 1 }}>
        <ReactFlow
          nodes={flowNodes}
          edges={flowEdges}
          nodeTypes={nodeTypes}
          fitView
          fitViewOptions={{ padding: 0.2 }}
          style={{ background: "#0f172a" }}
          onNodeClick={handleNodeClick}
          onEdgeClick={handleEdgeClick}
          onPaneClick={handlePaneClick}
          selectNodesOnDrag={false}
        >
          <Background color="#1e293b" gap={24} />
          <Controls style={{ background: "#1e293b", borderColor: "#334155", color: "#94a3b8" }} />
          <MiniMap
            style={{ background: "#1e293b" }}
            nodeColor={(n) => (n.type === "topicBridge"
              ? "#334155"
              : n.type === "externalEndpoint"
                ? "#44337a"
                : n.type === "semanticEndpoint"
                  ? "#f59e0b"
                  : "#3b82f6")}
          />
        </ReactFlow>
      </div>
      {selectedRosNode && (
        <NodeDetailPanel node={selectedRosNode} allNodes={allNodes} onClose={clearSelection} />
      )}
      {!selectedRosNode && selectedFlowNode?.type === "topicBridge" && (
        <TopicDetailPanel
          topic={(selectedFlowNode.data as { topic: string; msgType: string; visibilityState: TopicBridgeVisibility; semanticPeerIds?: string[] }).topic}
          msgType={(selectedFlowNode.data as { topic: string; msgType: string; visibilityState: TopicBridgeVisibility; semanticPeerIds?: string[] }).msgType}
          visibilityState={(selectedFlowNode.data as { topic: string; msgType: string; visibilityState: TopicBridgeVisibility; semanticPeerIds?: string[] }).visibilityState ?? "internal"}
          semanticPeerIds={(selectedFlowNode.data as { topic: string; msgType: string; visibilityState: TopicBridgeVisibility; semanticPeerIds?: string[] }).semanticPeerIds ?? []}
          allNodes={allNodes}
          messages={data.messages}
          onClose={clearSelection}
        />
      )}
      {!selectedRosNode && selectedFlowNode?.type === "externalEndpoint" && (
        <ExternalEndpointDetailPanel
          name={(selectedFlowNode.data as { name: string; endpointType: "action"|"service"; role: "server"|"client"; presence: EndpointPresence; semanticPeerIds?: string[] }).name}
          endpointType={(selectedFlowNode.data as { name: string; endpointType: "action"|"service"; role: "server"|"client"; presence: EndpointPresence; semanticPeerIds?: string[] }).endpointType}
          role={(selectedFlowNode.data as { name: string; endpointType: "action"|"service"; role: "server"|"client"; presence: EndpointPresence; semanticPeerIds?: string[] }).role}
          presence={(selectedFlowNode.data as { name: string; endpointType: "action"|"service"; role: "server"|"client"; presence: EndpointPresence; semanticPeerIds?: string[] }).presence}
          semanticPeerIds={(selectedFlowNode.data as { name: string; endpointType: "action"|"service"; role: "server"|"client"; presence: EndpointPresence; semanticPeerIds?: string[] }).semanticPeerIds ?? []}
          allNodes={allNodes}
          messages={data.messages}
          onClose={clearSelection}
        />
      )}
      {!selectedRosNode && selectedFlowNode?.type === "semanticEndpoint" && (
        <SemanticEndpointDetailPanel
          endpointType={(selectedFlowNode.data as { endpointType: "action" | "service"; labels: string[]; sourceId: string; targetId: string }).endpointType}
          labels={(selectedFlowNode.data as { endpointType: "action" | "service"; labels: string[]; sourceId: string; targetId: string }).labels}
          sourceId={(selectedFlowNode.data as { endpointType: "action" | "service"; labels: string[]; sourceId: string; targetId: string }).sourceId}
          targetId={(selectedFlowNode.data as { endpointType: "action" | "service"; labels: string[]; sourceId: string; targetId: string }).targetId}
          allNodes={allNodes}
          onClose={clearSelection}
        />
      )}
      {selectedEdge?.data && (
        <EdgeDetailPanel
          edgeData={selectedEdge.data as ServiceEdgeData | ActionEdgeData}
          allNodes={allNodes}
          messages={data.messages}
          onClose={() => setSelectedEdge(null)}
        />
      )}
    </div>
  );
}

// ── Node detail panel ─────────────────────────────────────────────────────────
function NodeDetailPanel({ node, allNodes, onClose }: { node: RosNode; allNodes: RosNode[]; onClose: () => void }) {
  const [tab, setTab] = useState<"actions" | "services" | "topics">("actions");

  // Build lookup maps for match detection
  const actionServerMap = useMemo(() => {
    const m = new Map<string, string>(); // action → node.id of server
    for (const n of allNodes) for (const a of n.actions) if (a.role === "server") m.set(a.action, n.id);
    return m;
  }, [allNodes]);
  const actionClientMap = useMemo(() => {
    const m = new Map<string, string[]>();
    for (const n of allNodes) for (const a of n.actions) if (a.role === "client") {
      const arr = m.get(a.action) ?? []; arr.push(n.id); m.set(a.action, arr);
    }
    return m;
  }, [allNodes]);
  const svcServerMap = useMemo(() => {
    const m = new Map<string, string>();
    for (const n of allNodes) for (const s of n.services) if (s.role === "server") m.set(s.service, n.id);
    return m;
  }, [allNodes]);
  const svcClientMap = useMemo(() => {
    const m = new Map<string, string[]>();
    for (const n of allNodes) for (const s of n.services) if (s.role === "client") {
      const arr = m.get(s.service) ?? []; arr.push(n.id); m.set(s.service, arr);
    }
    return m;
  }, [allNodes]);
  const topicPubMap = useMemo(() => {
    const m = new Map<string, string>(); // topic → publisher node.id
    for (const n of allNodes) for (const t of n.topics) if (t.direction === "pub") m.set(t.topic, n.id);
    return m;
  }, [allNodes]);
  const topicSubMap = useMemo(() => {
    const m = new Map<string, string[]>();
    for (const n of allNodes) for (const t of n.topics) if (t.direction === "sub") {
      const arr = m.get(t.topic) ?? []; arr.push(n.id); m.set(t.topic, arr);
    }
    return m;
  }, [allNodes]);

  const tabStyle = (active: boolean): React.CSSProperties => ({
    padding: "4px 10px", fontSize: 11, cursor: "pointer", borderRadius: 4,
    background: active ? "#1e293b" : "transparent",
    color: active ? "#e2e8f0" : "#64748b",
    border: `1px solid ${active ? "#334155" : "transparent"}`,
  });

  const matchedTag = (matched: boolean, partnerName?: string, semantic = false) => (
    <span style={{
      fontSize: 9, borderRadius: 3, padding: "1px 5px", marginLeft: 4, flexShrink: 0,
      background: matched ? (semantic ? "#211406" : "#052e16") : "#1c1028",
      border: `1px solid ${matched ? (semantic ? "#f59e0b" : "#166534") : "#3b1f5e"}`,
      color: matched ? (semantic ? "#fcd34d" : "#4ade80") : "#a78bfa",
    }}>
      {matched ? (partnerName ? `${semantic ? "≈" : "↔"} ${partnerName.split("/").pop()}` : semantic ? "≈ semantic" : "✓ matched") : "? external"}
    </span>
  );

  return (
    <div style={{
      width: 300, borderLeft: "1px solid #1e293b", background: "#080e1a",
      overflowY: "auto", flexShrink: 0, display: "flex", flexDirection: "column",
    }}>
      {/* Header */}
      <div style={{ padding: "10px 14px 6px", borderBottom: "1px solid #1e293b", flexShrink: 0 }}>
        <div style={{ display: "flex", alignItems: "center", justifyContent: "space-between" }}>
          <div style={{ fontFamily: "monospace", fontSize: 13, fontWeight: 700, color: "#e2e8f0" }}>
            {node.nodeName}
          </div>
          <button onClick={onClose} style={{ background: "none", border: "none", color: "#475569", cursor: "pointer", fontSize: 16, lineHeight: 1 }}>✕</button>
        </div>
        <div style={{ fontSize: 10, color: "#475569", marginTop: 2 }}>{node.package}</div>
        {node.sourceKind === "knownTool" && (
          <span style={{ fontSize: 9, background: "#132134", border: "1px solid #2563eb", borderRadius: 3, padding: "1px 5px", color: "#93c5fd", marginTop: 4, display: "inline-block", marginRight: 4 }}>
            launch-derived integration
          </span>
        )}
        {node.lifecycleNode && (
          <span style={{ fontSize: 9, background: "#0d2a1e", border: "1px solid #10b981", borderRadius: 3, padding: "1px 5px", color: "#34d399", marginTop: 4, display: "inline-block" }}>
            lifecycle
          </span>
        )}
        {node.derivedFromLaunches && node.derivedFromLaunches.length > 0 && (
          <div style={{ marginTop: 8, display: "grid", gap: 4 }}>
            <div style={{ fontSize: 10, color: "#94a3b8" }}>Launch sources</div>
            {node.derivedFromLaunches.map((launchPath) => (
              <div key={launchPath} style={{ fontFamily: "monospace", fontSize: 10, color: "#64748b", wordBreak: "break-all" }}>
                {launchPath}
              </div>
            ))}
          </div>
        )}
        {node.notes && node.notes.length > 0 && (
          <div style={{ marginTop: 8, display: "grid", gap: 4 }}>
            {node.notes.map((note) => (
              <div key={note} style={{ fontSize: 10, color: "#94a3b8", lineHeight: 1.4 }}>
                {note}
              </div>
            ))}
          </div>
        )}
      </div>

      {/* Tabs */}
      <div style={{ display: "flex", gap: 4, padding: "6px 10px", borderBottom: "1px solid #1e293b", flexShrink: 0 }}>
        <button style={tabStyle(tab === "actions")} onClick={() => setTab("actions")}>
          <Zap size={10} style={{ display: "inline", marginRight: 3 }} />Actions ({node.actions.length})
        </button>
        <button style={tabStyle(tab === "services")} onClick={() => setTab("services")}>
          <Wrench size={10} style={{ display: "inline", marginRight: 3 }} />Services ({node.services.length})
        </button>
        <button style={tabStyle(tab === "topics")} onClick={() => setTab("topics")}>
          Topics ({node.topics.length})
        </button>
      </div>

      {/* Content */}
      <div style={{ flex: 1, padding: "8px 14px", overflowY: "auto" }}>
        {tab === "actions" && (
          node.actions.length === 0
            ? <div style={{ fontSize: 11, color: "#475569", marginTop: 8 }}>No actions.</div>
            : node.actions.map((a, i) => {
                const exactPartnerId = a.role === "server"
                  ? (actionClientMap.get(a.action) ?? [])[0]
                  : actionServerMap.get(a.action);
                const semanticPartner = !exactPartnerId
                  ? findSemanticActionPeers(node, a.action, a.actionType, a.role === "server" ? "client" : "server", allNodes)[0]
                  : undefined;
                const partner = exactPartnerId
                  ? allNodes.find((n) => n.id === exactPartnerId)
                  : semanticPartner;
                return (
                  <div key={i} style={{ marginBottom: 10, paddingBottom: 10, borderBottom: "1px solid #0f1a2e" }}>
                    <div style={{ display: "flex", alignItems: "flex-start", gap: 4 }}>
                      <span style={{
                        fontSize: 9, padding: "1px 5px", borderRadius: 3, flexShrink: 0, marginTop: 1,
                        background: a.role === "server" ? "#1c1005" : "#0a1828",
                        border: `1px solid ${a.role === "server" ? "#92400e" : "#1e3a5f"}`,
                        color: a.role === "server" ? "#fcd34d" : "#60a5fa",
                      }}>
                        {a.role === "server" ? <><ArrowLeft size={8} style={{display:"inline"}} /> server</> : <><ArrowRight size={8} style={{display:"inline"}} /> client</>}
                      </span>
                      <div style={{ flex: 1, minWidth: 0 }}>
                        <div style={{ fontFamily: "monospace", fontSize: 11, color: "#fcd34d", wordBreak: "break-all" }}>{a.action}</div>
                        <div style={{ fontSize: 10, color: "#64748b" }}>{a.actionType}</div>
                      </div>
                    </div>
                    <div style={{ marginTop: 4, display: "flex", alignItems: "center" }}>
                      {matchedTag(!!partner, partner?.nodeName, !exactPartnerId && !!semanticPartner)}
                      {!partner && (
                        <span style={{ fontSize: 10, color: "#57534e", marginLeft: 4 }}>
                          {a.role === "server" ? "→ waiting for client (e.g. MoveIt, ros2_control)" : "→ looking for server"}
                        </span>
                      )}
                    </div>
                  </div>
                );
              })
        )}

        {tab === "services" && (
          node.services.length === 0
            ? <div style={{ fontSize: 11, color: "#475569", marginTop: 8 }}>No services.</div>
            : node.services.map((s, i) => {
                const exactPartnerId = s.role === "server"
                  ? (svcClientMap.get(s.service) ?? [])[0]
                  : svcServerMap.get(s.service);
                const semanticPartner = !exactPartnerId
                  ? findSemanticServicePeers(node, s.service, s.srvType, s.role === "server" ? "client" : "server", allNodes)[0]
                  : undefined;
                const partner = exactPartnerId
                  ? allNodes.find((n) => n.id === exactPartnerId)
                  : semanticPartner;
                return (
                  <div key={i} style={{ marginBottom: 10, paddingBottom: 10, borderBottom: "1px solid #0f1a2e" }}>
                    <div style={{ display: "flex", alignItems: "flex-start", gap: 4 }}>
                      <span style={{
                        fontSize: 9, padding: "1px 5px", borderRadius: 3, flexShrink: 0, marginTop: 1,
                        background: s.role === "server" ? "#1a0f30" : "#0a1828",
                        border: `1px solid ${s.role === "server" ? "#44337a" : "#1e3a5f"}`,
                        color: s.role === "server" ? "#d8b4fe" : "#a5b4fc",
                      }}>
                        {s.role === "server" ? <><ArrowLeft size={8} style={{display:"inline"}} /> server</> : <><ArrowRight size={8} style={{display:"inline"}} /> client</>}
                      </span>
                      <div style={{ flex: 1, minWidth: 0 }}>
                        <div style={{ fontFamily: "monospace", fontSize: 11, color: "#d8b4fe", wordBreak: "break-all" }}>{s.service}</div>
                        <div style={{ fontSize: 10, color: "#64748b" }}>{s.srvType}</div>
                      </div>
                    </div>
                    <div style={{ marginTop: 4 }}>{matchedTag(!!partner, partner?.nodeName, !exactPartnerId && !!semanticPartner)}</div>
                  </div>
                );
              })
        )}

        {tab === "topics" && (
          node.topics.length === 0
            ? <div style={{ fontSize: 11, color: "#475569", marginTop: 8 }}>No topics.</div>
            : node.topics.map((t, i) => {
                const isMatched = t.direction === "pub"
                  ? (topicSubMap.get(t.topic) ?? []).length > 0
                  : !!topicPubMap.get(t.topic);
                const semanticPartner = !isMatched
                  ? findSemanticTopicPeers(node, t.topic, t.msgType, t.direction === "pub" ? "sub" : "pub", allNodes)[0]
                  : undefined;
                return (
                  <div key={i} style={{ marginBottom: 8, paddingBottom: 8, borderBottom: "1px solid #0f1a2e" }}>
                    <div style={{ display: "flex", alignItems: "flex-start", gap: 4 }}>
                      <span style={{
                        fontSize: 9, padding: "1px 5px", borderRadius: 3, flexShrink: 0, marginTop: 1,
                        background: t.direction === "pub" ? "#0c2336" : "#052e16",
                        border: `1px solid ${t.direction === "pub" ? "#1e4a6e" : "#166534"}`,
                        color: t.direction === "pub" ? "#67e8f9" : "#86efac",
                      }}>
                        {t.direction === "pub" ? "▲ pub" : "▼ sub"}
                      </span>
                      <div style={{ flex: 1, minWidth: 0 }}>
                        <div style={{ fontFamily: "monospace", fontSize: 11, color: "#e2e8f0", wordBreak: "break-all" }}>{t.topic}</div>
                        <div style={{ fontSize: 10, color: "#64748b" }}>{t.msgType}</div>
                      </div>
                    </div>
                    <div style={{ marginTop: 3 }}>{matchedTag(isMatched || !!semanticPartner, semanticPartner?.nodeName, !isMatched && !!semanticPartner)}</div>
                  </div>
                );
              })
        )}
      </div>
    </div>
  );
}

// ── Topic detail panel ────────────────────────────────────────────────────────
function TopicDetailPanel({ topic, msgType, visibilityState, semanticPeerIds, allNodes, messages, onClose }: {
  topic: string; msgType: string; visibilityState: TopicBridgeVisibility;
  semanticPeerIds: string[];
  allNodes: RosNode[]; messages: MsgDef[]; onClose: () => void;
}) {
  const publishers = allNodes.filter((n) => n.topics.some((t) => t.direction === "pub" && t.topic === topic));
  const subscribers = allNodes.filter((n) => n.topics.some((t) => t.direction === "sub" && t.topic === topic));
  const semanticPeers = allNodes.filter((node) => semanticPeerIds.includes(node.id));
  const msgDef = findMsgDef(messages, msgType);

  return (
    <div style={{ width: 300, borderLeft: "1px solid #1e293b", background: "#080e1a", overflowY: "auto", flexShrink: 0, display: "flex", flexDirection: "column" }}>
      <div style={{ padding: "10px 14px 8px", borderBottom: "1px solid #1e293b", flexShrink: 0 }}>
        <div style={{ display: "flex", alignItems: "flex-start", justifyContent: "space-between", gap: 6 }}>
          <div style={{ fontFamily: "monospace", fontSize: 12, fontWeight: 700, color: "#e2e8f0", wordBreak: "break-all" }}>{topic}</div>
          <button onClick={onClose} style={{ background: "none", border: "none", color: "#475569", cursor: "pointer", fontSize: 16, lineHeight: 1, flexShrink: 0 }}>✕</button>
        </div>
        <div style={{ display: "flex", gap: 4, marginTop: 6, flexWrap: "wrap" }}>
          <span style={{ fontSize: 10, background: "#1e1b4b", border: "1px solid #4338ca", borderRadius: 4, padding: "1px 6px", color: "#a5b4fc" }}>
            {msgType || "unknown type"}
          </span>
          {visibilityState === "external" && (
            <span style={{ fontSize: 10, background: "#1c1028", border: "1px solid #7c3aed", borderRadius: 4, padding: "1px 6px", color: "#c084fc" }}>
              external source
            </span>
          )}
          {visibilityState === "filtered" && (
            <span style={{ fontSize: 10, background: "#0b1b2a", border: "1px solid #38bdf8", borderRadius: 4, padding: "1px 6px", color: "#7dd3fc" }}>
              peer hidden by filters
            </span>
          )}
          {visibilityState === "semantic" && (
            <span style={{ fontSize: 10, background: "#211406", border: "1px solid #f59e0b", borderRadius: 4, padding: "1px 6px", color: "#fcd34d" }}>
              semantic peer visible
            </span>
          )}
        </div>
      </div>
      <div style={{ padding: "8px 14px", overflowY: "auto", flex: 1 }}>
        {semanticPeers.length > 0 && (
          <div style={{ marginBottom: 12 }}>
            <div style={{ fontSize: 10, color: "#fcd34d", fontWeight: 700, marginBottom: 6, textTransform: "uppercase", letterSpacing: 0.5 }}>
              Semantic Peers ({semanticPeers.length})
            </div>
            {semanticPeers.map((node) => (
              <div key={node.id} style={{ fontSize: 11, color: "#e2e8f0", fontFamily: "monospace", padding: "2px 0" }}>
                <span style={{ color: pkgColor(node.package), fontSize: 9 }}>■ </span>
                {node.nodeName}<span style={{ color: "#475569" }}> · {node.package}</span>
              </div>
            ))}
            <div style={{ fontSize: 10, color: "#64748b", marginTop: 4 }}>
              Namespace-normalized match, not an exact raw topic name.
            </div>
          </div>
        )}
        <div style={{ marginBottom: 12 }}>
          <div style={{ fontSize: 10, color: "#67e8f9", fontWeight: 700, marginBottom: 6, textTransform: "uppercase", letterSpacing: 0.5 }}>
            ▲ Publishers ({publishers.length})
          </div>
          {publishers.length === 0
            ? <div style={{ fontSize: 11, color: "#475569" }}>None in workspace</div>
            : publishers.map((n) => (
              <div key={n.id} style={{ fontSize: 11, color: "#e2e8f0", fontFamily: "monospace", padding: "2px 0" }}>
                <span style={{ color: pkgColor(n.package), fontSize: 9 }}>■ </span>
                {n.nodeName}<span style={{ color: "#475569" }}> · {n.package}</span>
              </div>
            ))
          }
        </div>
        <div style={{ marginBottom: 12 }}>
          <div style={{ fontSize: 10, color: "#86efac", fontWeight: 700, marginBottom: 6, textTransform: "uppercase", letterSpacing: 0.5 }}>
            ▼ Subscribers ({subscribers.length})
          </div>
          {subscribers.length === 0
            ? <div style={{ fontSize: 11, color: "#475569" }}>None in workspace</div>
            : subscribers.map((n) => (
              <div key={n.id} style={{ fontSize: 11, color: "#e2e8f0", fontFamily: "monospace", padding: "2px 0" }}>
                <span style={{ color: pkgColor(n.package), fontSize: 9 }}>■ </span>
                {n.nodeName}<span style={{ color: "#475569" }}> · {n.package}</span>
              </div>
            ))
          }
        </div>
        {msgDef && <MsgDefSection msgDef={msgDef} />}
      </div>
    </div>
  );
}

// ── External endpoint detail panel ───────────────────────────────────────────
function ExternalEndpointDetailPanel({ name, endpointType, role, presence, semanticPeerIds, allNodes, messages, onClose }: {
  name: string; endpointType: "action" | "service"; role: "server" | "client"; presence: EndpointPresence; semanticPeerIds: string[];
  allNodes: RosNode[]; messages: MsgDef[]; onClose: () => void;
}) {
  // `role` is the MISSING side — find nodes with the PRESENT side (opposite role)
  const presentRole = role === "client" ? "server" : "client";
  const presentNodes = endpointType === "service"
    ? allNodes.filter((n) => n.services.some((s) => s.service === name && s.role === presentRole))
    : allNodes.filter((n) => n.actions.some((a) => a.action === name && a.role === presentRole));
  const semanticPeers = allNodes.filter((node) => semanticPeerIds.includes(node.id));
  const typeStr = presentNodes.length > 0
    ? (endpointType === "service"
        ? presentNodes[0].services.find((s) => s.service === name)?.srvType ?? ""
        : presentNodes[0].actions.find((a) => a.action === name)?.actionType ?? "")
    : "";
  const msgDef = typeStr ? findMsgDef(messages, typeStr) : undefined;
  const isAction = endpointType === "action";
  const accentColor = isAction ? "#fcd34d" : "#d8b4fe";

  return (
    <div style={{ width: 300, borderLeft: "1px solid #1e293b", background: "#080e1a", overflowY: "auto", flexShrink: 0, display: "flex", flexDirection: "column" }}>
      <div style={{ padding: "10px 14px 8px", borderBottom: "1px solid #1e293b", flexShrink: 0 }}>
        <div style={{ display: "flex", alignItems: "flex-start", justifyContent: "space-between", gap: 6 }}>
          <div style={{ fontFamily: "monospace", fontSize: 12, fontWeight: 700, color: "#e2e8f0", wordBreak: "break-all" }}>{name}</div>
          <button onClick={onClose} style={{ background: "none", border: "none", color: "#475569", cursor: "pointer", fontSize: 16, lineHeight: 1, flexShrink: 0 }}>✕</button>
        </div>
        <div style={{ display: "flex", gap: 4, marginTop: 6, flexWrap: "wrap" }}>
          <span style={{ fontSize: 10, background: isAction ? "#1c1500" : "#1a0f30", border: `1px solid ${accentColor}`, borderRadius: 4, padding: "1px 6px", color: accentColor }}>
            {isAction ? <Zap size={9} style={{ display: "inline", marginRight: 2 }} /> : <Wrench size={9} style={{ display: "inline", marginRight: 2 }} />}
            {endpointType}
          </span>
          <span style={{ fontSize: 10, background: presence === "semantic" ? "#211406" : presence === "filtered" ? "#0b1b2a" : "#1c1028", border: `1px solid ${presence === "semantic" ? "#f59e0b" : presence === "filtered" ? "#38bdf8" : "#7c3aed"}`, borderRadius: 4, padding: "1px 6px", color: presence === "semantic" ? "#fcd34d" : presence === "filtered" ? "#7dd3fc" : "#c084fc" }}>
            {presence === "semantic" ? "semantic peer visible" : presence === "filtered" ? "hidden by filters" : "external to scan"}
          </span>
          {typeStr && (
            <span style={{ fontSize: 10, background: "#1e1b4b", border: "1px solid #4338ca", borderRadius: 4, padding: "1px 6px", color: "#a5b4fc" }}>
              {typeStr}
            </span>
          )}
        </div>
      </div>
      <div style={{ padding: "8px 14px", overflowY: "auto", flex: 1 }}>
        {semanticPeers.length > 0 && (
          <div style={{ marginBottom: 12 }}>
            <div style={{ fontSize: 10, color: "#fcd34d", fontWeight: 700, marginBottom: 6, textTransform: "uppercase", letterSpacing: 0.5 }}>
              Semantic Peers ({semanticPeers.length})
            </div>
            {semanticPeers.map((node) => (
              <div key={node.id} style={{ fontSize: 11, color: "#e2e8f0", fontFamily: "monospace", padding: "2px 0" }}>
                <span style={{ color: pkgColor(node.package), fontSize: 9 }}>■ </span>
                {node.nodeName}<span style={{ color: "#475569" }}> · {node.package}</span>
              </div>
            ))}
            <div style={{ fontSize: 10, color: "#64748b", marginTop: 4 }}>
              Exact ROS names differ, but the endpoint family matches after namespace normalization.
            </div>
          </div>
        )}
        <div style={{ marginBottom: 12 }}>
          <div style={{ fontSize: 10, color: accentColor, fontWeight: 700, marginBottom: 6, textTransform: "uppercase", letterSpacing: 0.5 }}>
            {presentRole === "server" ? "▼ Server" : "▲ Client"} ({presentNodes.length})
          </div>
          {presentNodes.map((n) => (
            <div key={n.id} style={{ fontSize: 11, color: "#e2e8f0", fontFamily: "monospace", padding: "2px 0" }}>
              <span style={{ color: pkgColor(n.package), fontSize: 9 }}>■ </span>
              {n.nodeName}<span style={{ color: "#475569" }}> · {n.package}</span>
            </div>
          ))}
        </div>
        <div style={{ marginBottom: 12 }}>
          <div style={{ fontSize: 10, color: "#64748b", fontWeight: 700, marginBottom: 4, textTransform: "uppercase", letterSpacing: 0.5 }}>
            {role === "client" ? "Client" : "Server"} — {presence === "semantic" ? "semantic peer visible" : presence === "filtered" ? "hidden by filters" : "external"}
          </div>
          <div style={{ fontSize: 11, color: "#475569", fontStyle: "italic" }}>
            {presence === "semantic"
              ? "A visible counterpart exists, but the match only appears after namespace normalization between helper and adapter endpoints."
              : presence === "filtered"
              ? "A matching counterpart exists in the workspace scan, but it is currently excluded by package or node filters."
              : (isAction
                  ? (role === "client" ? "Waiting for action client (e.g. MoveIt, ros2_control)" : "Looking for action server")
                  : (role === "client" ? "Waiting for service client" : "Looking for service server"))}
          </div>
        </div>
        {msgDef && <MsgDefSection msgDef={msgDef} />}
      </div>
    </div>
  );
}

function SemanticEndpointDetailPanel({ endpointType, labels, sourceId, targetId, allNodes, onClose }: {
  endpointType: "action" | "service";
  labels: string[];
  sourceId: string;
  targetId: string;
  allNodes: RosNode[];
  onClose: () => void;
}) {
  const isAction = endpointType === "action";
  const accentColor = isAction ? "#fcd34d" : "#f59e0b";
  const sourceNode = allNodes.find((node) => node.id === sourceId) ?? null;
  const targetNode = allNodes.find((node) => node.id === targetId) ?? null;

  return (
    <div style={{ width: 300, borderLeft: "1px solid #1e293b", background: "#080e1a", overflowY: "auto", flexShrink: 0, display: "flex", flexDirection: "column" }}>
      <div style={{ padding: "10px 14px 8px", borderBottom: "1px solid #1e293b", flexShrink: 0 }}>
        <div style={{ display: "flex", alignItems: "flex-start", justifyContent: "space-between", gap: 6 }}>
          <div style={{ fontFamily: "monospace", fontSize: 12, fontWeight: 700, color: "#e2e8f0" }}>
            semantic {endpointType}
          </div>
          <button onClick={onClose} style={{ background: "none", border: "none", color: "#475569", cursor: "pointer", fontSize: 16, lineHeight: 1, flexShrink: 0 }}>✕</button>
        </div>
        <div style={{ display: "flex", gap: 4, marginTop: 6, flexWrap: "wrap" }}>
          <span style={{ fontSize: 10, background: isAction ? "#1c1500" : "#211406", border: `1px solid ${accentColor}`, borderRadius: 4, padding: "1px 6px", color: accentColor }}>
            {labels.length} {labels.length === 1 ? "match" : "matches"}
          </span>
        </div>
      </div>
      <div style={{ padding: "8px 14px", overflowY: "auto", flex: 1 }}>
        <div style={{ marginBottom: 12 }}>
          <div style={{ fontSize: 10, color: accentColor, fontWeight: 700, marginBottom: 6, textTransform: "uppercase", letterSpacing: 0.5 }}>
            Endpoints
          </div>
          {labels.map((label) => (
            <div key={label} style={{ fontSize: 11, color: "#e2e8f0", fontFamily: "monospace", padding: "2px 0" }}>
              {label}
            </div>
          ))}
        </div>
        <div style={{ marginBottom: 12 }}>
          <div style={{ fontSize: 10, color: "#67e8f9", fontWeight: 700, marginBottom: 6, textTransform: "uppercase", letterSpacing: 0.5 }}>
            Source
          </div>
          {sourceNode ? (
            <div style={{ fontSize: 11, color: "#e2e8f0", fontFamily: "monospace" }}>
              <span style={{ color: pkgColor(sourceNode.package), fontSize: 9 }}>■ </span>
              {sourceNode.nodeName}<span style={{ color: "#475569" }}> · {sourceNode.package}</span>
            </div>
          ) : (
            <div style={{ fontSize: 11, color: "#475569" }}>Unknown</div>
          )}
        </div>
        <div style={{ marginBottom: 12 }}>
          <div style={{ fontSize: 10, color: "#86efac", fontWeight: 700, marginBottom: 6, textTransform: "uppercase", letterSpacing: 0.5 }}>
            Target
          </div>
          {targetNode ? (
            <div style={{ fontSize: 11, color: "#e2e8f0", fontFamily: "monospace" }}>
              <span style={{ color: pkgColor(targetNode.package), fontSize: 9 }}>■ </span>
              {targetNode.nodeName}<span style={{ color: "#475569" }}> · {targetNode.package}</span>
            </div>
          ) : (
            <div style={{ fontSize: 11, color: "#475569" }}>Unknown</div>
          )}
        </div>
        <div style={{ fontSize: 11, color: "#64748b", fontStyle: "italic" }}>
          This bridge exists because the ROS names match only after namespace normalization, so the relation is semantic rather than a direct raw-name match.
        </div>
      </div>
    </div>
  );
}

// ── Edge detail panel (matched service / action) ──────────────────────────────
type ServiceEdgeData = { kind: "service"; service: string; srvType: string; clientId: string; serverId: string };
type ActionEdgeData  = { kind: "action";  action: string; actionType: string; clientId: string; serverId: string };

function EdgeDetailPanel({ edgeData, allNodes, messages, onClose }: {
  edgeData: ServiceEdgeData | ActionEdgeData; allNodes: RosNode[]; messages: MsgDef[]; onClose: () => void;
}) {
  const isAction = edgeData.kind === "action";
  const name     = isAction ? edgeData.action  : edgeData.service;
  const typeStr  = isAction ? edgeData.actionType : edgeData.srvType;
  const clientNode = allNodes.find((n) => n.id === edgeData.clientId);
  const serverNode = allNodes.find((n) => n.id === edgeData.serverId);
  const msgDef   = findMsgDef(messages, typeStr);
  const accentColor = isAction ? "#fcd34d" : "#d8b4fe";

  return (
    <div style={{ width: 300, borderLeft: "1px solid #1e293b", background: "#080e1a", overflowY: "auto", flexShrink: 0, display: "flex", flexDirection: "column" }}>
      <div style={{ padding: "10px 14px 8px", borderBottom: "1px solid #1e293b", flexShrink: 0 }}>
        <div style={{ display: "flex", alignItems: "flex-start", justifyContent: "space-between", gap: 6 }}>
          <div style={{ fontFamily: "monospace", fontSize: 12, fontWeight: 700, color: "#e2e8f0", wordBreak: "break-all" }}>{name}</div>
          <button onClick={onClose} style={{ background: "none", border: "none", color: "#475569", cursor: "pointer", fontSize: 16, lineHeight: 1, flexShrink: 0 }}>✕</button>
        </div>
        <div style={{ display: "flex", gap: 4, marginTop: 6, flexWrap: "wrap" }}>
          <span style={{ fontSize: 10, background: isAction ? "#1c1500" : "#1a0f30", border: `1px solid ${accentColor}`, borderRadius: 4, padding: "1px 6px", color: accentColor }}>
            {isAction ? <Zap size={9} style={{ display: "inline", marginRight: 2 }} /> : <Wrench size={9} style={{ display: "inline", marginRight: 2 }} />}
            {edgeData.kind}
          </span>
          {typeStr && (
            <span style={{ fontSize: 10, background: "#1e1b4b", border: "1px solid #4338ca", borderRadius: 4, padding: "1px 6px", color: "#a5b4fc" }}>
              {typeStr}
            </span>
          )}
        </div>
      </div>
      <div style={{ padding: "8px 14px", overflowY: "auto", flex: 1 }}>
        <div style={{ marginBottom: 10 }}>
          <div style={{ fontSize: 10, color: accentColor, fontWeight: 700, marginBottom: 4 }}>
            <ArrowLeft size={10} style={{ display: "inline", marginRight: 3 }} />Server
          </div>
          {serverNode
            ? <NodeRef node={serverNode} />
            : <div style={{ fontSize: 11, color: "#475569" }}>{edgeData.serverId}</div>
          }
        </div>
        <div style={{ marginBottom: 12 }}>
          <div style={{ fontSize: 10, color: "#60a5fa", fontWeight: 700, marginBottom: 4 }}>
            <ArrowRight size={10} style={{ display: "inline", marginRight: 3 }} />Client
          </div>
          {clientNode
            ? <NodeRef node={clientNode} />
            : <div style={{ fontSize: 11, color: "#475569" }}>{edgeData.clientId}</div>
          }
        </div>
        {msgDef && <MsgDefSection msgDef={msgDef} />}
      </div>
    </div>
  );
}

// ── Shared sub-components ─────────────────────────────────────────────────────
function NodeRef({ node }: { node: RosNode }) {
  return (
    <div style={{ fontSize: 11, fontFamily: "monospace", color: "#e2e8f0", padding: "2px 0" }}>
      <span style={{ color: pkgColor(node.package), fontSize: 9 }}>■ </span>
      {node.nodeName}
      <div style={{ fontSize: 10, color: "#475569", marginLeft: 12 }}>{node.package}</div>
    </div>
  );
}

function MsgDefSection({ msgDef }: { msgDef: MsgDef }) {
  const kindLabel = msgDef.kind === "srv" ? "Service" : msgDef.kind === "action" ? "Action" : "Message";
  return (
    <div>
      <div style={{ fontSize: 10, color: "#94a3b8", fontWeight: 700, marginBottom: 4, textTransform: "uppercase", letterSpacing: 0.5 }}>
        {kindLabel} Fields
      </div>
      <div style={{ fontSize: 9, color: "#475569", marginBottom: 6 }}>{msgDef.package}/{msgDef.name}</div>
      {msgDef.fields.length === 0
        ? <div style={{ fontSize: 11, color: "#475569" }}>No fields defined.</div>
        : msgDef.fields.map((f, i) => (
          <div key={i} style={{ fontSize: 10, fontFamily: "monospace", padding: "2px 0", display: "flex", gap: 6 }}>
            <span style={{ color: "#60a5fa" }}>{f.type}</span>
            <span style={{ color: "#e2e8f0" }}>{f.name}</span>
          </div>
        ))
      }
    </div>
  );
}
