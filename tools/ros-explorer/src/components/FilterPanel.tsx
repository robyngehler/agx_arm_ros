import { useRef, useState, useEffect } from "react";
import { useStore } from "../store";
import { buildKnownToolNodes, TOOL_INTEGRATION_OPTIONS } from "../knownToolIntegrations";
import { Network, GitBranch, Activity, Search, SlidersHorizontal, MessageSquare } from "lucide-react";

type MultiSelectOption = {
  value: string;
  label: string;
  description?: string;
  searchText?: string;
};

// ── Custom multi-select dropdown ─────────────────────────────────────────────
function MultiSelectDropdown({
  label,
  icon,
  options,
  selected,
  onToggle,
  onClear,
  accentColor = "#3b82f6",
  placeholder,
  selectionSummary,
}: {
  label: string;
  icon: React.ReactNode;
  options: MultiSelectOption[];
  selected: Set<string>;
  onToggle: (v: string) => void;
  onClear: () => void;
  accentColor?: string;
  placeholder?: string;
  selectionSummary?: (count: number, total: number) => string;
}) {
  const [open, setOpen] = useState(false);
  const [search, setSearch] = useState("");
  const ref = useRef<HTMLDivElement>(null);

  useEffect(() => {
    if (!open) return;
    const handler = (e: MouseEvent) => {
      if (!ref.current?.contains(e.target as Node)) setOpen(false);
    };
    document.addEventListener("mousedown", handler);
    return () => document.removeEventListener("mousedown", handler);
  }, [open]);

  const filtered = options.filter((option) => {
    const haystack = `${option.label} ${option.description ?? ""} ${option.searchText ?? ""}`.toLowerCase();
    return haystack.includes(search.toLowerCase());
  });
  const count = options.filter((option) => selected.has(option.value)).length;
  const summary = selectionSummary
    ? selectionSummary(count, options.length)
    : count > 0
      ? `${count} ${label}${count > 1 ? "s" : ""} selected`
      : placeholder ?? `Filter ${label}…`;

  return (
    <div ref={ref} style={{ position: "relative" }}>
      {/* Trigger button */}
      <div
        onClick={() => setOpen(!open)}
        style={{
          display: "flex", alignItems: "center", gap: 6,
          background: "#1e293b", borderRadius: 6, padding: "5px 10px",
          border: `1px solid ${count > 0 ? accentColor : "#334155"}`,
          cursor: "pointer", userSelect: "none", minWidth: 155,
        }}
      >
        {icon}
        <span style={{ fontSize: 12, color: count > 0 ? "#e2e8f0" : "#64748b", flex: 1 }}>
          {summary}
        </span>
        {count > 0 && (
          <span
            onClick={(e) => { e.stopPropagation(); onClear(); }}
            style={{ color: "#475569", cursor: "pointer", fontSize: 15, lineHeight: 1 }}
          >×</span>
        )}
        <span style={{ color: "#475569", fontSize: 9 }}>{open ? "▲" : "▼"}</span>
      </div>

      {/* Dropdown panel — absolutely positioned below trigger */}
      {open && (
        <div style={{
          position: "absolute", top: "calc(100% + 4px)", left: 0, zIndex: 9999,
          background: "#0f172a", border: "1px solid #334155", borderRadius: 8,
          boxShadow: "0 8px 32px rgba(0,0,0,0.6)", minWidth: 280, maxHeight: 300,
          display: "flex", flexDirection: "column",
        }}>
          <div style={{ padding: "8px 10px", borderBottom: "1px solid #1e293b", flexShrink: 0 }}>
            <input
              autoFocus
              value={search}
              onChange={(e) => setSearch(e.target.value)}
              placeholder={`Search ${options.length} options…`}
              style={{
                width: "100%", boxSizing: "border-box",
                background: "#1e293b", border: "1px solid #334155",
                borderRadius: 4, padding: "4px 8px", color: "#e2e8f0", fontSize: 12, outline: "none",
              }}
            />
          </div>
          <div style={{ overflowY: "auto", flex: 1 }}>
            {filtered.length === 0 && (
              <div style={{ padding: "10px", color: "#475569", fontSize: 12 }}>No matches</div>
            )}
            {filtered.map((opt) => (
              <div
                key={opt.value}
                onClick={() => onToggle(opt.value)}
                style={{
                  display: "flex", alignItems: "center", gap: 8,
                  padding: "5px 10px", cursor: "pointer", fontSize: 12,
                  background: selected.has(opt.value) ? "#0f1f2a" : "transparent",
                  color: selected.has(opt.value) ? "#e2e8f0" : "#94a3b8",
                }}
              >
                <span style={{
                  width: 14, height: 14, borderRadius: 3, flexShrink: 0,
                  border: `1.5px solid ${selected.has(opt.value) ? accentColor : "#475569"}`,
                  background: selected.has(opt.value) ? accentColor : "transparent",
                  display: "flex", alignItems: "center", justifyContent: "center",
                  fontSize: 9, color: "#000", fontWeight: 900,
                }}>
                  {selected.has(opt.value) && "✓"}
                </span>
                <div style={{ minWidth: 0, display: "flex", flexDirection: "column" }}>
                  <span style={{ overflow: "hidden", textOverflow: "ellipsis", whiteSpace: "nowrap" }}>{opt.label}</span>
                  {opt.description && (
                    <span style={{ fontSize: 10, color: "#64748b", overflow: "hidden", textOverflow: "ellipsis", whiteSpace: "nowrap" }}>
                      {opt.description}
                    </span>
                  )}
                </div>
              </div>
            ))}
          </div>
          {count > 0 && (
            <div style={{
              padding: "6px 10px", borderTop: "1px solid #1e293b",
              display: "flex", justifyContent: "space-between", flexShrink: 0,
            }}>
              <span style={{ fontSize: 11, color: "#64748b" }}>{count} selected</span>
              <span onClick={onClear} style={{ fontSize: 11, color: "#3b82f6", cursor: "pointer" }}>Clear all</span>
            </div>
          )}
        </div>
      )}
    </div>
  );
}

// ── Button style helper ───────────────────────────────────────────────────────
const btnStyle = (active: boolean): React.CSSProperties => ({
  padding: "5px 14px",
  borderRadius: 6,
  border: `1px solid ${active ? "#3b82f6" : "#334155"}`,
  background: active ? "#1e3a5f" : "#0f172a",
  color: active ? "#60a5fa" : "#64748b",
  cursor: "pointer",
  fontSize: 12,
  fontWeight: 600,
  display: "flex",
  alignItems: "center",
  gap: 5,
});

export function FilterPanel() {
  const {
    activeView, data,
    showTopics, showServices, showActions,
    toolIntegrations,
    selectedPackages, topicFilters, msgTypeFilters,
    selectedNodeIds,
    setShowTopics, setShowServices, setShowActions,
    setToolIntegrationEnabled,
    togglePackage, toggleNodeSelection, setPackageNodeSelection,
    toggleTopicFilter, clearTopicFilters,
    toggleMsgTypeFilter, clearMsgTypeFilters,
  } = useStore();

  if (!data) return null;
  if (activeView === "launch") return null;

  const allNodes = data.nodes.concat(buildKnownToolNodes(data, toolIntegrations));

  const packagesWithNodes = data.packages.filter((p) =>
    data.nodes.some((n) => n.package === p.name)
  );

  const topicOptions = Array.from(new Set(
    allNodes.flatMap((n) => [
      ...n.topics.map((t) => t.topic),
      ...n.services.map((s) => s.service),
      ...n.actions.map((a) => a.action),
    ])
  )).sort().map((value) => ({ value, label: value }));

  const msgTypeOptions = Array.from(new Set(
    allNodes.flatMap((n) => [
      ...n.topics.map((t) => t.msgType),
      ...n.services.map((s) => s.srvType),
      ...n.actions.map((a) => a.actionType),
    ]).filter(Boolean)
  )).sort().map((value) => ({ value, label: value }));

  const packageNodeOptions = new Map(
    packagesWithNodes.map((pkg) => {
      const options = data.nodes
        .filter((node) => node.package === pkg.name)
        .sort((left, right) => {
          const leftPath = left.filePath.toLowerCase();
          const rightPath = right.filePath.toLowerCase();
          const leftPriority = Number(leftPath.includes("/test/") || leftPath.includes("/tests/") || left.nodeName.toLowerCase().includes("test") || left.nodeName.toLowerCase().includes("demo"));
          const rightPriority = Number(rightPath.includes("/test/") || rightPath.includes("/tests/") || right.nodeName.toLowerCase().includes("test") || right.nodeName.toLowerCase().includes("demo"));
          if (leftPriority !== rightPriority) return leftPriority - rightPriority;
          return left.nodeName.localeCompare(right.nodeName);
        })
        .map((node) => ({
          value: node.id,
          label: node.nodeName,
          description: node.filePath.split("/").slice(-2).join("/"),
          searchText: `${node.filePath} ${node.id}`,
        }));
      return [pkg.name, options] as const;
    }),
  );

  return (
    <div style={{
      display: "flex", alignItems: "center", gap: 12,
      padding: "8px 16px", background: "#0f172a",
      borderBottom: "1px solid #1e293b", flexWrap: "wrap", flexShrink: 0,
    }}>
      <SlidersHorizontal size={14} color="#475569" />

      {/* ── Graph filters ────────────────────────────────────────────────── */}
      {activeView === "graph" && (
        <>
          <button style={btnStyle(showTopics)} onClick={() => setShowTopics(!showTopics)}>
            <span style={{ background: "#164e63", borderRadius: 3, padding: "0 4px", color: "#67e8f9", fontSize: 10 }}>T</span>
            Topics
          </button>
          <button style={btnStyle(showServices)} onClick={() => setShowServices(!showServices)}>
            <span style={{ background: "#44337a", borderRadius: 3, padding: "0 4px", color: "#d8b4fe", fontSize: 10 }}>S</span>
            Services
          </button>
          <button style={btnStyle(showActions)} onClick={() => setShowActions(!showActions)}>
            <span style={{ background: "#78350f", borderRadius: 3, padding: "0 4px", color: "#fcd34d", fontSize: 10 }}>A</span>
            Actions
          </button>

          <div style={{ width: 1, height: 24, background: "#1e293b", margin: "0 4px" }} />

          <span style={{ fontSize: 11, color: "#64748b" }}>Integrations</span>
          {TOOL_INTEGRATION_OPTIONS.map((option) => {
            const enabled = toolIntegrations[option.key];
            return (
              <button
                key={option.key}
                title={option.description}
                style={btnStyle(enabled)}
                onClick={() => setToolIntegrationEnabled(option.key, !enabled)}
              >
                <span style={{ background: `${option.accentColor}22`, borderRadius: 3, padding: "0 4px", color: option.accentColor, fontSize: 10 }}>
                  {option.key === "moveit" ? "M" : "R"}
                </span>
                {option.label}
              </button>
            );
          })}

          {packagesWithNodes.map((p) => (
            <button key={p.name} style={btnStyle(selectedPackages.has(p.name))} onClick={() => togglePackage(p.name)}>
              {p.name}
            </button>
          ))}

          {packagesWithNodes.filter((pkg) => selectedPackages.has(pkg.name)).length > 0 && (
            <div style={{ flexBasis: "100%", height: 0 }} />
          )}

          {packagesWithNodes.filter((pkg) => selectedPackages.has(pkg.name)).map((pkg) => {
            const options = packageNodeOptions.get(pkg.name) ?? [];
            const selectedForPackage = new Set(
              options
                .filter((option) => selectedNodeIds.has(option.value))
                .map((option) => option.value),
            );
            return (
              <MultiSelectDropdown
                key={`${pkg.name}-nodes`}
                label="node"
                icon={<Network size={12} color={selectedForPackage.size < options.length ? "#60a5fa" : "#475569"} />}
                options={options}
                selected={selectedForPackage}
                onToggle={toggleNodeSelection}
                onClear={() => setPackageNodeSelection(pkg.name, [])}
                accentColor="#2563eb"
                placeholder={`${pkg.name} nodes…`}
                selectionSummary={(count, total) => `${pkg.name}: ${count}/${total} nodes`}
              />
            );
          })}

          <div style={{ width: 1, height: 24, background: "#1e293b", margin: "0 4px" }} />

          <MultiSelectDropdown
            label="topic"
            icon={<Search size={12} color={topicFilters.size > 0 ? "#67e8f9" : "#475569"} />}
            options={topicOptions}
            selected={topicFilters}
            onToggle={toggleTopicFilter}
            onClear={clearTopicFilters}
            accentColor="#0891b2"
          />
          <MultiSelectDropdown
            label="type"
            icon={<MessageSquare size={12} color={msgTypeFilters.size > 0 ? "#a78bfa" : "#475569"} />}
            options={msgTypeOptions}
            selected={msgTypeFilters}
            onToggle={toggleMsgTypeFilter}
            onClear={clearMsgTypeFilters}
            accentColor="#7c3aed"
          />
        </>
      )}
    </div>
  );
}

export function ViewTabs() {
  const { activeView, setActiveView } = useStore();
  const tabs = [
    { key: "graph" as const, label: "Node Graph", icon: <Network size={14} /> },
    { key: "launch" as const, label: "Launch Trees", icon: <GitBranch size={14} /> },
    { key: "lifecycle" as const, label: "Lifecycle / States", icon: <Activity size={14} /> },
  ];
  return (
    <div style={{ display: "flex", borderBottom: "1px solid #1e293b", background: "#0a0f1a", flexShrink: 0 }}>
      {tabs.map((t) => (
        <button key={t.key} onClick={() => setActiveView(t.key)} style={{
          display: "flex", alignItems: "center", gap: 6, padding: "10px 20px",
          background: "none", border: "none",
          borderBottom: activeView === t.key ? "2px solid #3b82f6" : "2px solid transparent",
          color: activeView === t.key ? "#60a5fa" : "#64748b",
          cursor: "pointer", fontSize: 13, fontWeight: activeView === t.key ? 700 : 400,
        }}>
          {t.icon}{t.label}
        </button>
      ))}
    </div>
  );
}


