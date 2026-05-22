import { useStore } from "./store";
import { ViewTabs, FilterPanel } from "./components/FilterPanel";
import { WorkspaceLoader } from "./components/WorkspaceLoader";
import { NodeGraphView } from "./views/NodeGraphView";
import { LaunchTreeView } from "./views/LaunchTreeView";
import { LifecycleView } from "./views/LifecycleView";
import "./App.css";

function App() {
  const { activeView } = useStore();

  return (
    <div
      style={{
        display: "flex",
        flexDirection: "column",
        height: "100vh",
        background: "#0a0f1a",
        color: "#e2e8f0",
        fontFamily: "'Inter', system-ui, sans-serif",
        overflow: "hidden",
      }}
    >
      <div
        style={{
          display: "flex",
          alignItems: "center",
          padding: "0 16px",
          height: 44,
          background: "#030712",
          borderBottom: "1px solid #1e293b",
          flexShrink: 0,
          gap: 12,
        }}
      >
        <span style={{ fontSize: 15, fontWeight: 700, color: "#60a5fa", letterSpacing: "0.03em" }}>
          ROS Explorer
        </span>
        <span style={{ fontSize: 11, color: "#334155" }}>agx_arm_ros</span>
      </div>
      <WorkspaceLoader />
      <ViewTabs />
      <FilterPanel />
      <div style={{ flex: 1, overflow: "hidden", position: "relative" }}>
        {activeView === "graph" && <NodeGraphView />}
        {activeView === "launch" && <LaunchTreeView />}
        {activeView === "lifecycle" && <LifecycleView />}
      </div>
    </div>
  );
}

export default App;
