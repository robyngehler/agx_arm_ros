import { create } from "zustand";
import type { WorkspaceData } from "./types";

interface ExplorerState {
  data: WorkspaceData | null;
  loading: boolean;
  error: string | null;
  activeView: "graph" | "launch" | "lifecycle";

  // node graph filters
  showTopics: boolean;
  showServices: boolean;
  showActions: boolean;
  selectedPackages: Set<string>;
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
  togglePackage: (pkg: string) => void;
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
  selectedPackages: new Set(),
  topicFilters: new Set<string>(),
  msgTypeFilters: new Set<string>(),
  launchDepthLimit: 3,
  selectedLaunchEntry: null,
  selectedLaunchFile: null,
  selectedLifecycleNode: null,

  setData: (data) => set({ data, selectedPackages: new Set(data.packages.map((p) => p.name)) }),
  setLoading: (loading) => set({ loading }),
  setError: (error) => set({ error }),
  setActiveView: (activeView) => set({ activeView }),
  setShowTopics: (showTopics) => set({ showTopics }),
  setShowServices: (showServices) => set({ showServices }),
  setShowActions: (showActions) => set({ showActions }),
  togglePackage: (pkg) => {
    const s = new Set(get().selectedPackages);
    if (s.has(pkg)) s.delete(pkg);
    else s.add(pkg);
    set({ selectedPackages: s });
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
