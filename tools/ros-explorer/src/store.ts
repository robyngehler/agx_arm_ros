import { create } from "zustand";
import type { ToolIntegrationKey, WorkspaceData } from "./types";

interface ExplorerState {
  data: WorkspaceData | null;
  loading: boolean;
  error: string | null;
  activeView: "graph" | "launch" | "lifecycle";

  // node graph filters
  showTopics: boolean;
  showServices: boolean;
  showActions: boolean;
  toolIntegrations: Record<ToolIntegrationKey, boolean>;
  selectedPackages: Set<string>;
  selectedNodeIds: Set<string>;
  topicFilters: Set<string>;
  msgTypeFilters: Set<string>;

  // launch tree
  launchDepthLimit: number;
  selectedLaunchEntry: string | null;
  selectedLaunchFile: string | null;

  // lifecycle filters
  selectedLifecycleNode: string | null;

  setData: (data: WorkspaceData) => void;
  setLoading: (v: boolean) => void;
  setError: (e: string | null) => void;
  setActiveView: (v: ExplorerState["activeView"]) => void;
  setShowTopics: (v: boolean) => void;
  setShowServices: (v: boolean) => void;
  setShowActions: (v: boolean) => void;
  setToolIntegrationEnabled: (key: ToolIntegrationKey, enabled: boolean) => void;
  togglePackage: (pkg: string) => void;
  toggleNodeSelection: (nodeId: string) => void;
  setPackageNodeSelection: (pkg: string, nodeIds: string[]) => void;
  toggleTopicFilter: (v: string) => void;
  clearTopicFilters: () => void;
  toggleMsgTypeFilter: (v: string) => void;
  clearMsgTypeFilters: () => void;
  setLaunchDepthLimit: (v: number) => void;
  setSelectedLaunchEntry: (id: string | null) => void;
  setSelectedLaunchFile: (id: string | null) => void;
  setSelectedLifecycleNode: (id: string | null) => void;
}

export const useStore = create<ExplorerState>((set, get) => ({
  data: null,
  loading: false,
  error: null,
  activeView: "graph",
  showTopics: true,
  showServices: true,
  showActions: true,
  toolIntegrations: {
    moveit: true,
    rviz: true,
  },
  selectedPackages: new Set(),
  selectedNodeIds: new Set(),
  topicFilters: new Set<string>(),
  msgTypeFilters: new Set<string>(),
  launchDepthLimit: 3,
  selectedLaunchEntry: null,
  selectedLaunchFile: null,
  selectedLifecycleNode: null,

  setData: (data) => set({
    data,
    selectedPackages: new Set(data.packages.map((p) => p.name)),
    selectedNodeIds: new Set(data.nodes.map((n) => n.id)),
  }),
  setLoading: (loading) => set({ loading }),
  setError: (error) => set({ error }),
  setActiveView: (activeView) => set({ activeView }),
  setShowTopics: (showTopics) => set({ showTopics }),
  setShowServices: (showServices) => set({ showServices }),
  setShowActions: (showActions) => set({ showActions }),
  setToolIntegrationEnabled: (key, enabled) => set({
    toolIntegrations: {
      ...get().toolIntegrations,
      [key]: enabled,
    },
  }),
  togglePackage: (pkg) => {
    const s = new Set(get().selectedPackages);
    if (s.has(pkg)) s.delete(pkg);
    else s.add(pkg);
    set({ selectedPackages: s });
  },
  toggleNodeSelection: (nodeId) => {
    const selectedNodeIds = new Set(get().selectedNodeIds);
    if (selectedNodeIds.has(nodeId)) selectedNodeIds.delete(nodeId);
    else selectedNodeIds.add(nodeId);
    set({ selectedNodeIds });
  },
  setPackageNodeSelection: (pkg, nodeIds) => {
    const data = get().data;
    if (!data) return;
    const packageNodeIds = new Set(
      data.nodes.filter((node) => node.package === pkg).map((node) => node.id),
    );
    const selectedNodeIds = new Set(get().selectedNodeIds);
    for (const nodeId of packageNodeIds) {
      selectedNodeIds.delete(nodeId);
    }
    for (const nodeId of nodeIds) {
      selectedNodeIds.add(nodeId);
    }
    set({ selectedNodeIds });
  },
  toggleTopicFilter: (v) => {
    const s = new Set(get().topicFilters);
    if (s.has(v)) s.delete(v); else s.add(v);
    set({ topicFilters: s });
  },
  clearTopicFilters: () => set({ topicFilters: new Set() }),
  toggleMsgTypeFilter: (v) => {
    const s = new Set(get().msgTypeFilters);
    if (s.has(v)) s.delete(v); else s.add(v);
    set({ msgTypeFilters: s });
  },
  clearMsgTypeFilters: () => set({ msgTypeFilters: new Set() }),
  setLaunchDepthLimit: (launchDepthLimit) => set({ launchDepthLimit }),
  setSelectedLaunchEntry: (selectedLaunchEntry) => set({ selectedLaunchEntry, selectedLaunchFile: null }),
  setSelectedLaunchFile: (selectedLaunchFile) => set({ selectedLaunchFile }),
  setSelectedLifecycleNode: (selectedLifecycleNode) => set({ selectedLifecycleNode }),
}));
