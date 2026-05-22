import { useMemo, useState, useCallback } from "react";
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
import { Zap, Wrench, ArrowRight, ArrowLeft, TriangleRight } from "lucide-react";
import { useStore } from "../store";
import type { RosNode, MsgDef } from "../types";

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
  const { topic, msgType, isExternal = false } = data as {
    topic: string;
    msgType: string;
    isExternal?: boolean;
  };
  return (
    <div
      style={{
        border: isExternal ? "1px dashed #6366f1" : "1px dashed #475569",
        borderRadius: 8,
        background: isExternal ? "#1e1b4b" : "#0f172a",
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
      {isExternal && (
        <div style={{ fontSize: 9, color: "#818cf8", marginBottom: 1 }}>external</div>
      )}
      <div style={{ fontWeight: 600, color: isExternal ? "#a5b4fc" : "#e2e8f0" }}>{topic}</div>
      <div style={{ color: "#64748b", fontSize: 10 }}>{msgType}</div>
    </div>
  );
}

// ── External action / service endpoint bridge ────────────────────────────────
function ExternalEndpointCard({ data }: NodeProps) {
  const { name, endpointType, role } = data as {
    name: string;
    endpointType: "action" | "service";
    role: "server" | "client";  // the MISSING side
  };
  const isAction = endpointType === "action";
  const color = isAction ? "#fcd34d" : "#d8b4fe";
  const bg = isAction ? "#1c1500" : "#1e1028";
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
        {endpointType} {role} (external)
      </div>
      <div style={{ fontWeight: 600, color: "#e2e8f0", wordBreak: "break-all" }}>{name}</div>
    </div>
  );
}

const nodeTypes = { rosNode: RosNodeCard, topicBridge: TopicBridgeCard, externalEndpoint: ExternalEndpointCard };

// ── Dagre auto-layout (left-to-right hierarchical) ───────────────────────────
const NODE_W = 240;
const NODE_H = 100;
const BRIDGE_W = 180;
const BRIDGE_H = 54;

function applyDagreLayout(nodes: Node[], edges: Edge[]): Node[] {
  if (nodes.length === 0) return nodes;
  const g = new Dagre.graphlib.Graph().setDefaultEdgeLabel(() => ({}));
  g.setGraph({ rankdir: "LR", nodesep: 60, ranksep: 120, edgesep: 20 });
  for (const n of nodes) {
    g.setNode(n.id, {
      width: n.type === "topicBridge" ? BRIDGE_W : NODE_W,
      height: n.type === "topicBridge" ? BRIDGE_H : NODE_H,
    });
  }
  for (const e of edges) {
    if (g.hasNode(e.source) && g.hasNode(e.target)) {
      g.setEdge(e.source, e.target);
    }
  }
  Dagre.layout(g);
  return nodes.map((n) => {
    const pos = g.node(n.id);
    const w = n.type === "topicBridge" ? BRIDGE_W : NODE_W;
    const h = n.type === "topicBridge" ? BRIDGE_H : NODE_H;
    return { ...n, position: { x: pos.x - w / 2, y: pos.y - h / 2 } };
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

// ── Main view ─────────────────────────────────────────────────────────────────
export function NodeGraphView() {
  const { data, showTopics, showServices, showActions, selectedPackages, topicFilters, msgTypeFilters } = useStore();

  const [selectedItemId, setSelectedItemId] = useState<string | null>(null);
  const [selectedEdge, setSelectedEdge] = useState<Edge | null>(null);

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

    const rosNodes = data.nodes.filter((n) => selectedPackages.has(n.package));

    const rfNodes: Node[] = rosNodes.map((n) => ({
      id: n.id,
      type: "rosNode",
      position: { x: 0, y: 0 },
      data: { node: n },
    }));

    const rfEdges: Edge[] = [];
    const topicBridgeMap = new Map<string, string>(); // topic → bridge node id

    const topicPass = (name: string) => topicFilters.size === 0 || topicFilters.has(name);
    const typePass = (t: string) => msgTypeFilters.size === 0 || msgTypeFilters.has(t) || [...msgTypeFilters].some((f) => shortType(f) === shortType(t));

    if (showTopics) {
      // Pass 1: create bridge nodes for all published topics
      for (const n of rosNodes) {
        for (const t of n.topics) {
          if (t.direction !== "pub") continue;
          if (!topicPass(t.topic) || !typePass(t.msgType)) continue;
          if (!topicBridgeMap.has(t.topic)) {
            const bridgeId = `topic__${t.topic.replace(/\//g, "_")}`;
            topicBridgeMap.set(t.topic, bridgeId);
            rfNodes.push({
              id: bridgeId,
              type: "topicBridge",
              position: { x: 0, y: 0 },
              data: { topic: t.topic, msgType: t.msgType, isExternal: false },
            });
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
          if (!topicPass(t.topic) || !typePass(t.msgType)) continue;
          if (topicBridgeMap.has(t.topic)) continue; // publisher bridge already exists
          const bridgeId = `topic__${t.topic.replace(/\//g, "_")}`;
          topicBridgeMap.set(t.topic, bridgeId);
          rfNodes.push({
            id: bridgeId,
            type: "topicBridge",
            position: { x: 0, y: 0 },
            data: { topic: t.topic, msgType: t.msgType, isExternal: true },
          });
        }
      }
      // Pass 3: connect all subscribers to their bridge (internal or external)
      for (const n of rosNodes) {
        for (const t of n.topics) {
          if (t.direction !== "sub") continue;
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
      for (const [svcName, { nodeId: serverId }] of svcServers) {
        if ((svcClients.get(svcName) ?? []).length === 0) {
          const bridgeId = `svc_ext_c__${svcName.replace(/\//g, "_")}`;
          rfNodes.push({ id: bridgeId, type: "externalEndpoint", position: { x: 0, y: 0 },
            data: { name: svcName, endpointType: "service", role: "client" } });
          rfEdges.push({ id: `${serverId}--svc_ext--${svcName}`, source: serverId, target: bridgeId,
            style: { stroke: "#7c5caa", strokeDasharray: "3,4", strokeWidth: 1 },
            markerEnd: { type: MarkerType.ArrowClosed, color: "#7c5caa" } });
        }
      }
      // Unmatched client → external server bridge
      for (const [svcName, clientIds] of svcClients) {
        if (!svcServers.has(svcName)) {
          const bridgeId = `svc_ext_s__${svcName.replace(/\//g, "_")}`;
          rfNodes.push({ id: bridgeId, type: "externalEndpoint", position: { x: 0, y: 0 },
            data: { name: svcName, endpointType: "service", role: "server" } });
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
      for (const [actName, { nodeId: serverId }] of actServers) {
        if ((actClients.get(actName) ?? []).length === 0) {
          const bridgeId = `act_ext_c__${actName.replace(/\//g, "_")}`;
          rfNodes.push({ id: bridgeId, type: "externalEndpoint", position: { x: 0, y: 0 },
            data: { name: actName, endpointType: "action", role: "client" } });
          rfEdges.push({ id: `${serverId}--act_ext--${actName}`, source: serverId, target: bridgeId,
            style: { stroke: "#92700e", strokeDasharray: "4,4", strokeWidth: 1.5 },
            markerEnd: { type: MarkerType.ArrowClosed, color: "#92700e" } });
        }
      }
      // Unmatched client → external server bridge
      for (const [actName, clientIds] of actClients) {
        if (!actServers.has(actName)) {
          const bridgeId = `act_ext_s__${actName.replace(/\//g, "_")}`;
          rfNodes.push({ id: bridgeId, type: "externalEndpoint", position: { x: 0, y: 0 },
            data: { name: actName, endpointType: "action", role: "server" } });
          for (const clientId of clientIds) {
            rfEdges.push({ id: `${clientId}--act_ext--${actName}`, source: clientId, target: bridgeId,
              style: { stroke: "#92700e", strokeDasharray: "4,4", strokeWidth: 1.5 },
              markerEnd: { type: MarkerType.ArrowClosed, color: "#92700e" } });
          }
        }
      }
    }

    // ── Apply selection-based dimming ────────────────────────────────────────────
    const connectedIds = selectedItemId
      ? getConnectedIdsFromEdges(selectedItemId, rfEdges)
      : selectedEdge
        ? new Set([selectedEdge.source, selectedEdge.target])
        : null;

    const finalNodes = rfNodes.map((n) => ({
      ...n,
      data: { ...(n.data as object), isSelected: n.id === selectedItemId },
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
          ? (selectedEdge
              ? (e.id === selectedEdge.id ? 1 : 0.06)
              : (connectedIds.has(e.source) && connectedIds.has(e.target) ? 1 : 0.06))
          : 1,
        transition: "opacity 0.2s",
      },
    }));

    return { nodes: applyDagreLayout(finalNodes, finalEdges), edges: finalEdges };
  }, [data, showTopics, showServices, showActions, selectedPackages, topicFilters, msgTypeFilters, selectedItemId, selectedEdge]);

  if (!data) return <div style={{ padding: 40, color: "#94a3b8" }}>No workspace data loaded.</div>;

  const clearSelection = () => { setSelectedItemId(null); setSelectedEdge(null); };
  const selectedRosNode = selectedItemId ? data.nodes.find((n) => n.id === selectedItemId) ?? null : null;
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
            nodeColor={(n) => (n.type === "topicBridge" ? "#334155" : n.type === "externalEndpoint" ? "#44337a" : "#3b82f6")}
          />
        </ReactFlow>
      </div>
      {selectedRosNode && (
        <NodeDetailPanel node={selectedRosNode} allNodes={data.nodes} onClose={clearSelection} />
      )}
      {!selectedRosNode && selectedFlowNode?.type === "topicBridge" && (
        <TopicDetailPanel
          topic={(selectedFlowNode.data as { topic: string; msgType: string; isExternal: boolean }).topic}
          msgType={(selectedFlowNode.data as { topic: string; msgType: string; isExternal: boolean }).msgType}
          isExternal={(selectedFlowNode.data as { topic: string; msgType: string; isExternal: boolean }).isExternal ?? false}
          allNodes={data.nodes}
          messages={data.messages}
          onClose={clearSelection}
        />
      )}
      {!selectedRosNode && selectedFlowNode?.type === "externalEndpoint" && (
        <ExternalEndpointDetailPanel
          name={(selectedFlowNode.data as { name: string; endpointType: "action"|"service"; role: "server"|"client" }).name}
          endpointType={(selectedFlowNode.data as { name: string; endpointType: "action"|"service"; role: "server"|"client" }).endpointType}
          role={(selectedFlowNode.data as { name: string; endpointType: "action"|"service"; role: "server"|"client" }).role}
          allNodes={data.nodes}
          messages={data.messages}
          onClose={clearSelection}
        />
      )}
      {selectedEdge?.data && (
        <EdgeDetailPanel
          edgeData={selectedEdge.data as ServiceEdgeData | ActionEdgeData}
          allNodes={data.nodes}
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

  const matchedTag = (matched: boolean, partnerName?: string) => (
    <span style={{
      fontSize: 9, borderRadius: 3, padding: "1px 5px", marginLeft: 4, flexShrink: 0,
      background: matched ? "#052e16" : "#1c1028",
      border: `1px solid ${matched ? "#166534" : "#3b1f5e"}`,
      color: matched ? "#4ade80" : "#a78bfa",
    }}>
      {matched ? (partnerName ? `↔ ${partnerName.split("/").pop()}` : "✓ matched") : "? external"}
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
        {node.lifecycleNode && (
          <span style={{ fontSize: 9, background: "#0d2a1e", border: "1px solid #10b981", borderRadius: 3, padding: "1px 5px", color: "#34d399", marginTop: 4, display: "inline-block" }}>
            lifecycle
          </span>
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
                const partnerId = a.role === "server"
                  ? (actionClientMap.get(a.action) ?? [])[0]
                  : actionServerMap.get(a.action);
                const partner = partnerId ? allNodes.find((n) => n.id === partnerId) : undefined;
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
                      {matchedTag(!!partner, partner?.nodeName)}
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
                const partnerId = s.role === "server"
                  ? (svcClientMap.get(s.service) ?? [])[0]
                  : svcServerMap.get(s.service);
                const partner = partnerId ? allNodes.find((n) => n.id === partnerId) : undefined;
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
                    <div style={{ marginTop: 4 }}>{matchedTag(!!partner, partner?.nodeName)}</div>
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
                    <div style={{ marginTop: 3 }}>{matchedTag(isMatched)}</div>
                  </div>
                );
              })
        )}
      </div>
    </div>
  );
}

// ── Topic detail panel ────────────────────────────────────────────────────────
function TopicDetailPanel({ topic, msgType, isExternal, allNodes, messages, onClose }: {
  topic: string; msgType: string; isExternal: boolean;
  allNodes: RosNode[]; messages: MsgDef[]; onClose: () => void;
}) {
  const publishers = allNodes.filter((n) => n.topics.some((t) => t.direction === "pub" && t.topic === topic));
  const subscribers = allNodes.filter((n) => n.topics.some((t) => t.direction === "sub" && t.topic === topic));
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
          {isExternal && (
            <span style={{ fontSize: 10, background: "#1c1028", border: "1px solid #7c3aed", borderRadius: 4, padding: "1px 6px", color: "#c084fc" }}>
              external source
            </span>
          )}
        </div>
      </div>
      <div style={{ padding: "8px 14px", overflowY: "auto", flex: 1 }}>
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
function ExternalEndpointDetailPanel({ name, endpointType, role, allNodes, messages, onClose }: {
  name: string; endpointType: "action" | "service"; role: "server" | "client";
  allNodes: RosNode[]; messages: MsgDef[]; onClose: () => void;
}) {
  // `role` is the MISSING side — find nodes with the PRESENT side (opposite role)
  const presentRole = role === "client" ? "server" : "client";
  const presentNodes = endpointType === "service"
    ? allNodes.filter((n) => n.services.some((s) => s.service === name && s.role === presentRole))
    : allNodes.filter((n) => n.actions.some((a) => a.action === name && a.role === presentRole));
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
          {typeStr && (
            <span style={{ fontSize: 10, background: "#1e1b4b", border: "1px solid #4338ca", borderRadius: 4, padding: "1px 6px", color: "#a5b4fc" }}>
              {typeStr}
            </span>
          )}
        </div>
      </div>
      <div style={{ padding: "8px 14px", overflowY: "auto", flex: 1 }}>
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
            {role === "client" ? "Client" : "Server"} — external
          </div>
          <div style={{ fontSize: 11, color: "#475569", fontStyle: "italic" }}>
            {isAction
              ? (role === "client" ? "Waiting for action client (e.g. MoveIt, ros2_control)" : "Looking for action server")
              : (role === "client" ? "Waiting for service client" : "Looking for service server")}
          </div>
        </div>
        {msgDef && <MsgDefSection msgDef={msgDef} />}
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
