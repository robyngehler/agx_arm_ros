import { useRef, useState, useEffect } from "react";
import { useStore } from "../store";
import { Network, GitBranch, Activity, Search, SlidersHorizontal, MessageSquare } from "lucide-react";

// ── Custom multi-select dropdown ─────────────────────────────────────────────
function MultiSelectDropdown({
  label,
  icon,
  options,
  selected,
  onToggle,
  onClear,
  accentColor = "#3b82f6",
}: {
  label: string;
  icon: React.ReactNode;
  options: string[];
  selected: Set<string>;
  onToggle: (v: string) => void;
  onClear: () => void;
  accentColor?: string;
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

  const filtered = options.filter((o) => o.toLowerCase().includes(search.toLowerCase()));
  const count = selected.size;

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
          {count > 0 ? `${count} ${label}${count > 1 ? "s" : ""} selected` : `Filter ${label}…`}
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
                key={opt}
                onClick={() => onToggle(opt)}
                style={{
                  display: "flex", alignItems: "center", gap: 8,
                  padding: "5px 10px", cursor: "pointer", fontSize: 12,
                  background: selected.has(opt) ? "#0f1f2a" : "transparent",
                  color: selected.has(opt) ? "#e2e8f0" : "#94a3b8",
                }}
              >
                <span style={{
                  width: 14, height: 14, borderRadius: 3, flexShrink: 0,
                  border: `1.5px solid ${selected.has(opt) ? accentColor : "#475569"}`,
                  background: selected.has(opt) ? accentColor : "transparent",
                  display: "flex", alignItems: "center", justifyContent: "center",
                  fontSize: 9, color: "#000", fontWeight: 900,
                }}>
                  {selected.has(opt) && "✓"}
                </span>
                <span style={{ overflow: "hidden", textOverflow: "ellipsis", whiteSpace: "nowrap" }}>{opt}</span>
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
    selectedPackages, topicFilters, msgTypeFilters,
    setShowTopics, setShowServices, setShowActions,
    togglePackage,
    toggleTopicFilter, clearTopicFilters,
    toggleMsgTypeFilter, clearMsgTypeFilters,
  } = useStore();

  if (!data) return null;
  if (activeView === "launch") return null;

  const packagesWithNodes = data.packages.filter((p) =>
    data.nodes.some((n) => n.package === p.name)
  );

  const topicOptions = Array.from(new Set(
    data.nodes.flatMap((n) => [
      ...n.topics.map((t) => t.topic),
      ...n.services.map((s) => s.service),
      ...n.actions.map((a) => a.action),
    ])
  )).sort();

  const msgTypeOptions = Array.from(new Set(
    data.nodes.flatMap((n) => [
      ...n.topics.map((t) => t.msgType),
      ...n.services.map((s) => s.srvType),
      ...n.actions.map((a) => a.actionType),
    ]).filter(Boolean)
  )).sort();

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

          {packagesWithNodes.map((p) => (
            <button key={p.name} style={btnStyle(selectedPackages.has(p.name))} onClick={() => togglePackage(p.name)}>
              {p.name}
            </button>
          ))}

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


