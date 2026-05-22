import { useState, useMemo } from "react";
import {
  ChevronDown, ChevronRight, Terminal, FileText,
  Package, Settings, GitBranch, Play,
} from "lucide-react";
import { useStore } from "../store";
import type { LaunchFile, LaunchNodeEntry, LaunchArg, RosEntryPoint } from "../types";

// ── Launch category ───────────────────────────────────────────────────────────
type LaunchCategory = "main" | "viz" | "tool" | "infra";

const CATEGORY_STYLE: Record<LaunchCategory, { label: string; bg: string; border: string; color: string }> = {
  main:  { label: "ENTRY", bg: "#071d11", border: "#10b981", color: "#34d399" },
  viz:   { label: "VIZ",   bg: "#130d2e", border: "#7c3aed", color: "#a78bfa" },
  tool:  { label: "TOOL",  bg: "#1c1200", border: "#d97706", color: "#fbbf24" },
  infra: { label: "INFRA", bg: "#0e1118", border: "#475569", color: "#64748b" },
};

function categorizeLaunch(l: LaunchFile): LaunchCategory {
  const name = (l.filePath.split("/").pop() ?? "").toLowerCase();
  if (/setup_assistant/.test(name)) return "tool";
  if (/warehouse_db/.test(name)) return "tool";
  if (/view_model|moveit_rviz/.test(name)) return "viz";
  if (/display\.launch/.test(name) &&
      l.nodes.filter((n) => !/rviz/i.test(n.executable)).length === 0) return "viz";
  if (/static.*tf|spawn_controller/.test(name)) return "infra";
  if (/^rsp\./.test(name)) return "infra";
  const hasRealNodes = l.nodes.some((n) => !/rviz|robot_state_publisher/i.test(n.executable));
  if (hasRealNodes || l.includes.length >= 2) return "main";
  if (l.nodes.length > 0) return "main";
  if (l.includes.length === 0 && l.args.length === 0) return "infra";
  return "main";
}

function scoreLaunch(l: LaunchFile): number {
  return l.nodes.length * 10 + l.includes.length * 3 + l.args.length;
}

// ── Path tracing (breadcrumb) ─────────────────────────────────────────────────
function findIncludePath(
  current: LaunchFile,
  targetId: string,
  allLaunches: LaunchFile[],
  visited = new Set<string>(),
): LaunchFile[] | null {
  if (current.id === targetId) return [current];
  if (visited.has(current.id)) return null;
  visited.add(current.id);
  for (const inc of current.includes) {
    const child = allLaunches.find((l) => l.filePath.endsWith(inc.file.split("/").pop()!));
    if (child) {
      const path = findIncludePath(child, targetId, allLaunches, new Set(visited));
      if (path) return [current, ...path];
    }
  }
  return null;
}

// ── Arg table ─────────────────────────────────────────────────────────────────
function ArgTable({ args }: { args: LaunchArg[] }) {
  if (!args.length) return null;
  return (
    <table style={{ width: "100%", borderCollapse: "collapse", marginTop: 4, fontSize: 11 }}>
      <thead>
        <tr style={{ background: "#1e293b" }}>
          <th style={thStyle}>Name</th>
          <th style={thStyle}>Default</th>
          <th style={thStyle}>Description</th>
        </tr>
      </thead>
      <tbody>
        {args.map((a) => (
          <tr key={a.name} style={{ borderBottom: "1px solid #1e293b" }}>
            <td style={tdStyle}><code style={{ color: "#67e8f9" }}>{a.name}</code></td>
            <td style={tdStyle}><code style={{ color: "#fcd34d" }}>{a.default ?? "\u2014"}</code></td>
            <td style={{ ...tdStyle, color: "#94a3b8" }}>{a.description ?? ""}</td>
          </tr>
        ))}
      </tbody>
    </table>
  );
}

const thStyle: React.CSSProperties = { textAlign: "left", padding: "3px 8px", color: "#64748b", fontWeight: 600 };
const tdStyle: React.CSSProperties = { padding: "3px 8px", color: "#e2e8f0", verticalAlign: "top" };

// ── Compact node row ──────────────────────────────────────────────────────────
function NodeRow({ n }: { n: LaunchNodeEntry }) {
  return (
    <div style={{ display: "flex", alignItems: "center", gap: 6, padding: "2px 0", fontSize: 11 }}>
      <Terminal size={11} color="#10b981" style={{ flexShrink: 0 }} />
      <code style={{ color: "#34d399" }}>{n.package}</code>
      <span style={{ color: "#334155" }}>/</span>
      <code style={{ color: "#e2e8f0" }}>{n.executable}</code>
      {n.name && n.name !== n.executable && (
        <span style={{ color: "#64748b" }}>as <code style={{ color: "#fcd34d" }}>{n.name}</code></span>
      )}
      {n.condition && (
        <span style={{
          background: "#1c1005", border: "1px solid #57534e",
          borderRadius: 3, padding: "0 4px", fontSize: 9, color: "#a8a29e",
        }}>
          if {n.condition}
        </span>
      )}
    </div>
  );
}

// ── Tree node (recursive) ─────────────────────────────────────────────────────
function TreeNode({
  launch,
  allLaunches,
  depth,
  maxDepth,
  isLast,
  selectedId,
  onSelect,
  category,
}: {
  launch: LaunchFile;
  allLaunches: LaunchFile[];
  depth: number;
  maxDepth: number;
  isLast: boolean;
  selectedId: string | null;
  onSelect: (id: string) => void;
  category?: LaunchCategory; // shown as a badge only at depth 0
}) {
  const isSelected = launch.id === selectedId;
  const filename = launch.filePath.split("/").pop() ?? launch.id;

  const resolvedIncludes = launch.includes.map((inc) => ({
    inc,
    child: allLaunches.find((l) => l.filePath.endsWith(inc.file.split("/").pop()!)),
  }));
  const internalChildren = resolvedIncludes.filter(
    (rc): rc is { inc: typeof rc.inc; child: LaunchFile } => rc.child !== undefined,
  );
  const externalChildren = resolvedIncludes.filter((rc) => rc.child === undefined);
  const hasChildren = internalChildren.length > 0 || externalChildren.length > 0;
  const atDepthLimit = depth >= maxDepth;
  // A node is "dimmed" when it sits at the depth boundary and has hidden children
  const dimmed = atDepthLimit && hasChildren;

  const [expanded, setExpanded] = useState(depth < maxDepth);
  const [showArgs, setShowArgs] = useState(false);
  const [limitOverride, setLimitOverride] = useState(false);

  const showChildren = expanded && (!atDepthLimit || limitOverride);

  // Clicking the row selects the launch file (shows detail panel).
  // Clicking the chevron separately toggles expand/collapse.
  const handleRowClick = () => onSelect(launch.id);
  const handleChevronClick = (e: React.MouseEvent) => {
    e.stopPropagation();
    if (!atDepthLimit || limitOverride) setExpanded((v) => !v);
    else setLimitOverride(true); // first chevron-click at limit → override instead of collapse
  };

  // Connector-line color dims when the node is at the depth boundary
  const lineColor = dimmed ? "#162032" : "#1e3a5f";

  return (
    <div>
      <div style={{ display: "flex", alignItems: "flex-start" }}>
        {depth > 0 && (
          <div style={{ width: 20, minWidth: 20, alignSelf: "stretch", position: "relative" }}>
            <div style={{
              position: "absolute", left: 9, top: 0,
              bottom: isLast ? 18 : 0,
              width: 1, background: lineColor,
            }} />
            <div style={{ position: "absolute", left: 9, top: 17, width: 11, height: 1, background: lineColor }} />
          </div>
        )}

        <div style={{ flex: 1, marginBottom: 4 }}>
          <div
            onClick={handleRowClick}
            style={{
              display: "flex", alignItems: "center", gap: 6,
              background: isSelected ? "#0c1e35" : dimmed ? "#090e18" : "#0f172a",
              border: `1px solid ${isSelected ? "#3b82f6" : depth === 0 ? "#1e3a5f" : dimmed ? "#131c2e" : "#1e293b"}`,
              borderRadius: (showArgs || (expanded && launch.nodes.length > 0)) ? "6px 6px 0 0" : 6,
              padding: "5px 10px", cursor: "pointer", userSelect: "none",
              opacity: dimmed && !isSelected ? 0.55 : 1,
            }}
          >
            {/* Chevron — click independently to expand/collapse */}
            {hasChildren && !atDepthLimit ? (
              <span onClick={handleChevronClick} style={{ display: "flex", flexShrink: 0 }}>
                {expanded
                  ? <ChevronDown size={13} color="#475569" />
                  : <ChevronRight size={13} color="#475569" />}
              </span>
            ) : hasChildren && atDepthLimit && !limitOverride ? (
              // At limit with hidden children: show a muted "expand" hint
              <span onClick={handleChevronClick} style={{ display: "flex", flexShrink: 0, cursor: "pointer" }}>
                <ChevronRight size={13} color="#2d3f55" />
              </span>
            ) : (
              <span style={{ width: 13, flexShrink: 0 }} />
            )}

            <FileText
              size={13}
              color={isSelected ? "#3b82f6" : dimmed ? "#2d4060" : depth === 0 ? "#3b82f6" : "#475569"}
              style={{ flexShrink: 0 }}
            />

            {/* Category badge — only at root level */}
            {depth === 0 && category && !dimmed && (
              <span style={{
                background: CATEGORY_STYLE[category].bg,
                border: `1px solid ${CATEGORY_STYLE[category].border}`,
                borderRadius: 3, padding: "1px 6px", fontSize: 9, fontWeight: 700,
                color: CATEGORY_STYLE[category].color, flexShrink: 0, letterSpacing: "0.05em",
              }}>
                {CATEGORY_STYLE[category].label}
              </span>
            )}

            <span style={{
              fontFamily: "monospace", fontSize: 12, fontWeight: depth === 0 ? 700 : 500,
              color: isSelected ? "#93c5fd" : dimmed ? "#3d5070" : "#e2e8f0",
              flex: 1, minWidth: 0, overflow: "hidden", textOverflow: "ellipsis", whiteSpace: "nowrap",
            }}>
              {filename}
            </span>

            {/* Show a "(+N hidden)" badge when children are cut off */}
            {atDepthLimit && hasChildren && !limitOverride && (
              <span style={{
                background: "#0d1a2e", border: "1px solid #1e3a5f", borderRadius: 3,
                padding: "1px 5px", fontSize: 10, color: "#2d4f75", flexShrink: 0,
              }}>
                +{launch.includes.length} hidden
              </span>
            )}

            <span style={{
              display: "flex", alignItems: "center", gap: 3,
              background: dimmed ? "#0d1825" : "#1e3a5f",
              borderRadius: 3, padding: "1px 5px", fontSize: 10,
              color: dimmed ? "#2d4060" : "#60a5fa", flexShrink: 0,
            }}>
              <Package size={8} />{launch.package}
            </span>

            {launch.nodes.length > 0 && (
              <span style={{
                display: "flex", alignItems: "center", gap: 3,
                background: dimmed ? "#04130c" : "#052e16",
                borderRadius: 3, padding: "1px 5px", fontSize: 10,
                color: dimmed ? "#1a3a28" : "#34d399", flexShrink: 0,
              }}>
                <Terminal size={8} />{launch.nodes.length}
              </span>
            )}

            {launch.args.length > 0 && (
              <button
                onClick={(e) => { e.stopPropagation(); setShowArgs(!showArgs); }}
                style={{
                  background: showArgs ? "#1c1500" : "transparent",
                  border: `1px solid ${showArgs ? "#f59e0b" : "#334155"}`,
                  borderRadius: 3, padding: "1px 5px", fontSize: 10,
                  color: showArgs ? "#fcd34d" : dimmed ? "#2d3a4a" : "#64748b",
                  cursor: "pointer", flexShrink: 0, display: "flex", alignItems: "center", gap: 3,
                }}
              >
                <Settings size={8} />{launch.args.length} arg{launch.args.length !== 1 ? "s" : ""}
              </button>
            )}
          </div>

          {showArgs && (
            <div style={{
              background: "#07111e", border: "1px solid #1e293b", borderTop: "none",
              padding: "6px 10px",
              borderRadius: expanded && launch.nodes.length > 0 ? 0 : "0 0 5px 5px",
            }}>
              <ArgTable args={launch.args} />
            </div>
          )}

          {expanded && launch.nodes.length > 0 && (
            <div style={{
              background: "#07111e", border: "1px solid #1e293b", borderTop: "none",
              padding: "4px 10px 6px", borderRadius: "0 0 5px 5px",
            }}>
              {launch.nodes.map((n, i) => <NodeRow key={i} n={n} />)}
            </div>
          )}
        </div>
      </div>

      {expanded && (
        <div style={{ marginLeft: depth === 0 ? 24 : 20 + 24 }}>
          {showChildren ? (
            <>
              {internalChildren.map(({ inc, child }, i) => (
                <div key={child.id}>
                  {inc.condition && (
                    <div style={{ fontSize: 10, color: "#78716c", marginBottom: 2, marginLeft: 20 }}>
                      <span style={{ color: "#57534e" }}>if</span>{" "}
                      <code style={{ color: "#a8a29e" }}>{inc.condition}</code>
                    </div>
                  )}
                  <TreeNode
                    launch={child}
                    allLaunches={allLaunches}
                    depth={depth + 1}
                    maxDepth={maxDepth}
                    isLast={i === internalChildren.length - 1 && externalChildren.length === 0}
                    selectedId={selectedId}
                    onSelect={onSelect}
                  />
                </div>
              ))}

              {externalChildren.map(({ inc }, i) => (
                <div key={inc.file} style={{ display: "flex", alignItems: "flex-start", marginBottom: 4 }}>
                  <div style={{ width: 20, minWidth: 20, alignSelf: "stretch", position: "relative" }}>
                    <div style={{
                      position: "absolute", left: 9, top: 0,
                      bottom: i === externalChildren.length - 1 ? 18 : 0,
                      width: 1, background: lineColor,
                    }} />
                    <div style={{ position: "absolute", left: 9, top: 13, width: 11, height: 1, background: lineColor }} />
                  </div>
                  <div style={{ display: "flex", alignItems: "center", gap: 6, fontSize: 11, color: "#44403c" }}>
                    <FileText size={11} color="#44403c" />
                    <code>{inc.file.split("/").pop()}</code>
                    {inc.condition && <span style={{ color: "#57534e", fontSize: 10 }}>[if {inc.condition}]</span>}
                    <span style={{ fontSize: 10 }}>(external)</span>
                  </div>
                </div>
              ))}
            </>
          ) : atDepthLimit && hasChildren ? (
            <div
              onClick={() => setLimitOverride(true)}
              style={{
                marginLeft: 20, fontSize: 11, color: "#2d4060",
                cursor: "pointer", padding: "2px 0",
                display: "flex", alignItems: "center", gap: 5,
              }}
            >
              <ChevronRight size={11} color="#2d4060" />
              {launch.includes.length} include{launch.includes.length !== 1 ? "s" : ""} hidden
              <span style={{ color: "#3b82f6" }}>— show anyway</span>
            </div>
          ) : null}
        </div>
      )}
    </div>
  );
}

// ── Detail panel ──────────────────────────────────────────────────────────────
function DetailPanel({
  launch,
  entryLaunch,
  allLaunches,
}: {
  launch: LaunchFile;
  entryLaunch: LaunchFile | null;
  allLaunches: LaunchFile[];
}) {
  const breadcrumb = useMemo(
    () => entryLaunch ? findIncludePath(entryLaunch, launch.id, allLaunches) : [launch],
    [entryLaunch, launch, allLaunches],
  );

  return (
    <div style={{
      width: 280, borderLeft: "1px solid #1e293b",
      background: "#080e1a", overflowY: "auto", padding: 16, flexShrink: 0,
    }}>
      {breadcrumb && breadcrumb.length > 1 && (
        <div style={{ marginBottom: 12, display: "flex", flexWrap: "wrap", gap: 4, alignItems: "center" }}>
          {breadcrumb.map((b, i) => (
            <span key={b.id} style={{ display: "flex", alignItems: "center", gap: 4 }}>
              <span style={{ fontSize: 10, fontFamily: "monospace", color: i === breadcrumb.length - 1 ? "#e2e8f0" : "#64748b" }}>
                {b.filePath.split("/").pop()}
              </span>
              {i < breadcrumb.length - 1 && <span style={{ color: "#334155" }}>\u203a</span>}
            </span>
          ))}
        </div>
      )}

      <div style={{ fontWeight: 700, fontSize: 14, color: "#e2e8f0", marginBottom: 2, fontFamily: "monospace" }}>
        {launch.filePath.split("/").pop()}
      </div>
      <div style={{ fontSize: 11, color: "#475569", marginBottom: 12, display: "flex", alignItems: "center", gap: 4 }}>
        <Package size={10} />{launch.package}
      </div>

      <div style={{ display: "flex", flexWrap: "wrap", gap: 6, marginBottom: 14 }}>
        {launch.args.length > 0 && (
          <span style={{ background: "#1c1500", border: "1px solid #f59e0b", borderRadius: 4, padding: "2px 8px", fontSize: 11, color: "#fcd34d" }}>
            {launch.args.length} arg{launch.args.length !== 1 ? "s" : ""}
          </span>
        )}
        {launch.nodes.length > 0 && (
          <span style={{ background: "#052e16", border: "1px solid #10b981", borderRadius: 4, padding: "2px 8px", fontSize: 11, color: "#34d399" }}>
            {launch.nodes.length} node{launch.nodes.length !== 1 ? "s" : ""}
          </span>
        )}
        {launch.includes.length > 0 && (
          <span style={{ background: "#1e3a5f", border: "1px solid #3b82f6", borderRadius: 4, padding: "2px 8px", fontSize: 11, color: "#60a5fa" }}>
            {launch.includes.length} include{launch.includes.length !== 1 ? "s" : ""}
          </span>
        )}
      </div>

      <div style={{ fontSize: 10, color: "#334155", fontFamily: "monospace", marginBottom: 16, wordBreak: "break-all" }}>
        {launch.filePath}
      </div>

      {launch.args.length > 0 && (
        <>
          <div style={sectionLabel}>Arguments</div>
          <ArgTable args={launch.args} />
        </>
      )}

      {launch.nodes.length > 0 && (
        <>
          <div style={{ ...sectionLabel, marginTop: 16 }}>Nodes launched</div>
          {launch.nodes.map((n, i) => <div key={i} style={{ marginBottom: 4 }}><NodeRow n={n} /></div>)}
        </>
      )}

      {launch.includes.length > 0 && (
        <>
          <div style={{ ...sectionLabel, marginTop: 16 }}>Includes</div>
          {launch.includes.map((inc, i) => (
            <div key={i} style={{ marginBottom: 6 }}>
              <div style={{ fontFamily: "monospace", fontSize: 11, color: "#60a5fa" }}>
                {inc.file.split("/").pop()}
              </div>
              {inc.condition && <div style={{ fontSize: 10, color: "#64748b" }}>if {inc.condition}</div>}
              {inc.args && Object.keys(inc.args).length > 0 && (
                <div style={{ fontSize: 10, color: "#475569", fontFamily: "monospace", marginTop: 2 }}>
                  {Object.entries(inc.args).map(([k, v]) => <div key={k}>{k}={v}</div>)}
                </div>
              )}
            </div>
          ))}
        </>
      )}
    </div>
  );
}

const sectionLabel: React.CSSProperties = {
  fontSize: 10, fontWeight: 700, color: "#64748b",
  textTransform: "uppercase", letterSpacing: "0.06em", marginBottom: 6,
};

function stepBtn(active: boolean, disabled = false): React.CSSProperties {
  return {
    padding: "3px 8px", borderRadius: 4, fontSize: 11,
    cursor: disabled ? "default" : "pointer",
    border: `1px solid ${active ? "#3b82f6" : "#334155"}`,
    background: active ? "#1e3a5f" : "#0f172a",
    color: disabled ? "#334155" : active ? "#60a5fa" : "#64748b",
  };
}

// ── Sidebar helpers ───────────────────────────────────────────────────────────
function SidebarSectionHeader({ label, count }: { label: string; count: number }) {
  return (
    <div style={{
      padding: "10px 12px 4px",
      fontSize: 10, fontWeight: 700, color: "#64748b",
      textTransform: "uppercase", letterSpacing: "0.06em",
      display: "flex", alignItems: "center", justifyContent: "space-between",
    }}>
      {label}
      <span style={{ fontSize: 9, color: "#334155", fontWeight: 400, textTransform: "none" }}>{count}</span>
    </div>
  );
}

function SidebarLaunchItem({
  l, isActive, onClick, parents, category,
}: {
  l: LaunchFile;
  isActive: boolean;
  onClick: () => void;
  parents: string[];
  category?: LaunchCategory;
}) {
  const cs = category ? CATEGORY_STYLE[category] : null;
  return (
    <div
      onClick={onClick}
      style={{
        padding: "6px 12px", cursor: "pointer",
        borderLeft: `3px solid ${isActive ? "#3b82f6" : "transparent"}`,
        background: isActive ? "#0f1f3a" : "transparent",
      }}
    >
      <div style={{
        fontFamily: "monospace", fontSize: 11,
        color: isActive ? "#93c5fd" : "#94a3b8",
        overflow: "hidden", textOverflow: "ellipsis", whiteSpace: "nowrap",
      }}>
        {l.filePath.split("/").pop()}
      </div>
      <div style={{ display: "flex", alignItems: "center", gap: 4, marginTop: 2, flexWrap: "wrap" }}>
        {cs && (
          <span style={{
            fontSize: 8, fontWeight: 700, letterSpacing: "0.06em",
            background: cs.bg, border: `1px solid ${cs.border}`,
            color: cs.color, borderRadius: 3, padding: "0 4px",
          }}>
            {cs.label}
          </span>
        )}
        <span style={{ fontSize: 10, color: "#334155", display: "flex", alignItems: "center", gap: 2 }}>
          <Package size={8} />{l.package}
        </span>
        {parents.length > 0 && (
          <span
            title={`Included by: ${parents.join(", ")}`}
            style={{
              fontSize: 9, color: "#1e3a5f", background: "#0a1828",
              borderRadius: 3, padding: "0 4px", border: "1px solid #1e2d42",
            }}
          >
            ↑{parents.length}
          </span>
        )}
      </div>
    </div>
  );
}

// ── ros2 run entry detail panel ───────────────────────────────────────────────
function RunDetailPanel({ ep }: { ep: RosEntryPoint }) {
  return (
    <div style={{
      width: 280, borderLeft: "1px solid #1e293b",
      background: "#080e1a", overflowY: "auto", padding: 16, flexShrink: 0,
    }}>
      <div style={{ display: "flex", alignItems: "center", gap: 6, marginBottom: 12 }}>
        <Play size={13} color="#10b981" />
        <span style={{
          fontSize: 10, color: "#475569", textTransform: "uppercase",
          letterSpacing: "0.06em", fontWeight: 700,
        }}>
          ros2 run
        </span>
      </div>
      <div style={{ fontWeight: 700, fontSize: 14, color: "#34d399", marginBottom: 2, fontFamily: "monospace" }}>
        {ep.name}
      </div>
      <div style={{ fontSize: 11, color: "#475569", marginBottom: 16, display: "flex", alignItems: "center", gap: 4 }}>
        <Package size={10} />{ep.package}
      </div>
      <div style={{ ...sectionLabel }}>Module</div>
      <div style={{ fontFamily: "monospace", fontSize: 11, color: "#94a3b8", wordBreak: "break-all", marginBottom: 16 }}>
        {ep.module}
      </div>
      <div style={{
        fontFamily: "monospace", fontSize: 11, background: "#0a1828",
        border: "1px solid #1e293b", borderRadius: 6, padding: "6px 10px", color: "#60a5fa",
      }}>
        ros2 run {ep.package} {ep.name}
      </div>
    </div>
  );
}

// ── Main view ─────────────────────────────────────────────────────────────────
export function LaunchTreeView() {
  const {
    data,
    launchDepthLimit, setLaunchDepthLimit,
    selectedLaunchEntry, setSelectedLaunchEntry,
    selectedLaunchFile, setSelectedLaunchFile,
  } = useStore();

  // Local state — declared before any early returns
  const [selectedRosRun, setSelectedRosRun] = useState<RosEntryPoint | null>(null);

  // Build included-by map: filename → list of parent filenames
  const includedBy = useMemo(() => {
    if (!data) return new Map<string, string[]>();
    const map = new Map<string, string[]>();
    for (const l of data.launches) {
      for (const inc of l.includes) {
        const fname = inc.file.split("/").pop()!;
        const parents = map.get(fname) ?? [];
        parents.push(l.filePath.split("/").pop()!);
        map.set(fname, parents);
      }
    }
    return map;
  }, [data]);

  // Group root launches by category (sorted by score within each group)
  const grouped = useMemo(() => {
    if (!data) return { main: [] as LaunchFile[], viz: [] as LaunchFile[], toolInfra: [] as LaunchFile[] };
    const byScore = (a: LaunchFile, b: LaunchFile) => scoreLaunch(b) - scoreLaunch(a);
    const main: LaunchFile[] = [];
    const viz: LaunchFile[] = [];
    const toolInfra: LaunchFile[] = [];
    for (const l of data.launches) {
      if (includedBy.has(l.filePath.split("/").pop()!)) continue; // skip shared
      const cat = categorizeLaunch(l);
      if (cat === "main") main.push(l);
      else if (cat === "viz") viz.push(l);
      else toolInfra.push(l);
    }
    main.sort(byScore);
    viz.sort(byScore);
    toolInfra.sort(byScore);
    return { main, viz, toolInfra };
  }, [data, includedBy]);

  // Shared launches: included by at least one other launch in this workspace
  const sharedLaunches = useMemo(
    () => (data ? data.launches.filter((l) => includedBy.has(l.filePath.split("/").pop()!)) : []),
    [data, includedBy],
  );

  // Resolve the active entry (the one whose tree is displayed)
  const activeEntry = useMemo(() => {
    if (!data) return null;
    return (
      data.launches.find((l) => l.id === selectedLaunchEntry) ??
      grouped.main[0] ??
      grouped.viz[0] ??
      grouped.toolInfra[0] ??
      sharedLaunches[0] ??
      null
    );
  }, [data, selectedLaunchEntry, grouped, sharedLaunches]);

  const selectedFile = useMemo(
    () => (data && selectedLaunchFile ? (data.launches.find((l) => l.id === selectedLaunchFile) ?? null) : null),
    [data, selectedLaunchFile],
  );

  // Early return — all hooks are already called above
  if (!data) return <div style={{ padding: 40, color: "#94a3b8" }}>No workspace data loaded.</div>;

  const entryPoints = data.entryPoints ?? [];
  const totalRoot = grouped.main.length + grouped.viz.length + grouped.toolInfra.length;

  const handleLaunchClick = (id: string) => {
    setSelectedLaunchEntry(id);
    setSelectedRosRun(null);
  };

  const handleRosRunClick = (ep: RosEntryPoint) => {
    setSelectedRosRun((prev) =>
      prev?.name === ep.name && prev.package === ep.package ? null : ep
    );
    setSelectedLaunchFile(null);
  };

  return (
    <div style={{ display: "flex", height: "100%", overflow: "hidden" }}>

      {/* ── Entry-point sidebar ────────────────────────────────────────── */}
      <div style={{
        width: 210, borderRight: "1px solid #1e293b",
        overflowY: "auto", flexShrink: 0, background: "#080e1a",
      }}>
        <div style={{ fontSize: 10, color: "#334155", padding: "8px 12px 2px" }}>
          {totalRoot} root · {sharedLaunches.length} shared · {data.launches.length} total
        </div>

        {/* Entry Points section */}
        {grouped.main.length > 0 && (
          <>
            <SidebarSectionHeader label="Entry Points" count={grouped.main.length} />
            {grouped.main.map((l) => (
              <SidebarLaunchItem
                key={l.id} l={l} category="main"
                isActive={l.id === activeEntry?.id && !selectedRosRun}
                onClick={() => handleLaunchClick(l.id)}
                parents={includedBy.get(l.filePath.split("/").pop()!) ?? []}
              />
            ))}
          </>
        )}

        {/* Visualization section */}
        {grouped.viz.length > 0 && (
          <>
            <SidebarSectionHeader label="Visualization" count={grouped.viz.length} />
            {grouped.viz.map((l) => (
              <SidebarLaunchItem
                key={l.id} l={l} category="viz"
                isActive={l.id === activeEntry?.id && !selectedRosRun}
                onClick={() => handleLaunchClick(l.id)}
                parents={includedBy.get(l.filePath.split("/").pop()!) ?? []}
              />
            ))}
          </>
        )}

        {/* Tools & Setup section (at bottom of root section) */}
        {grouped.toolInfra.length > 0 && (
          <>
            <SidebarSectionHeader label="Tools & Setup" count={grouped.toolInfra.length} />
            {grouped.toolInfra.map((l) => (
              <SidebarLaunchItem
                key={l.id} l={l} category={categorizeLaunch(l)}
                isActive={l.id === activeEntry?.id && !selectedRosRun}
                onClick={() => handleLaunchClick(l.id)}
                parents={includedBy.get(l.filePath.split("/").pop()!) ?? []}
              />
            ))}
          </>
        )}

        {/* Shared launches (included by others) */}
        {sharedLaunches.length > 0 && (
          <>
            <div style={{
              margin: "8px 12px 4px", paddingTop: 8,
              borderTop: "1px solid #1e293b",
              fontSize: 10, fontWeight: 700, color: "#475569",
              textTransform: "uppercase", letterSpacing: "0.06em",
              display: "flex", alignItems: "center", gap: 4,
            }}>
              Shared
              <span style={{ fontSize: 9, color: "#334155", fontWeight: 400, textTransform: "none" }}>
                (also included)
              </span>
            </div>
            {sharedLaunches.map((l) => (
              <SidebarLaunchItem
                key={l.id} l={l}
                isActive={l.id === activeEntry?.id && !selectedRosRun}
                onClick={() => handleLaunchClick(l.id)}
                parents={includedBy.get(l.filePath.split("/").pop()!) ?? []}
              />
            ))}
          </>
        )}

        {/* ros2 run executables */}
        {entryPoints.length > 0 && (
          <>
            <div style={{
              margin: "8px 12px 4px", paddingTop: 8,
              borderTop: "1px solid #1e293b",
              fontSize: 10, fontWeight: 700, color: "#475569",
              textTransform: "uppercase", letterSpacing: "0.06em",
              display: "flex", alignItems: "center", gap: 4,
            }}>
              <Play size={9} color="#10b981" /> ros2 run
            </div>
            {entryPoints.map((ep) => {
              const isRunActive = selectedRosRun?.name === ep.name && selectedRosRun.package === ep.package;
              return (
                <div
                  key={`${ep.package}/${ep.name}`}
                  onClick={() => handleRosRunClick(ep)}
                  style={{
                    padding: "6px 12px", cursor: "pointer",
                    borderLeft: `3px solid ${isRunActive ? "#10b981" : "transparent"}`,
                    background: isRunActive ? "#061a0f" : "transparent",
                  }}
                >
                  <div style={{
                    fontFamily: "monospace", fontSize: 11,
                    color: isRunActive ? "#34d399" : "#94a3b8",
                    overflow: "hidden", textOverflow: "ellipsis", whiteSpace: "nowrap",
                  }}>
                    {ep.name}
                  </div>
                  <div style={{ fontSize: 10, color: "#334155", display: "flex", alignItems: "center", gap: 2, marginTop: 1 }}>
                    <Package size={8} />{ep.package}
                  </div>
                </div>
              );
            })}
          </>
        )}
      </div>

      {/* ── Tree panel ────────────────────────────────────────────────── */}
      <div style={{ flex: 1, display: "flex", flexDirection: "column", overflow: "hidden" }}>

        {/* Toolbar */}
        <div style={{
          padding: "6px 16px", borderBottom: "1px solid #1e293b",
          background: "#0a0f1a", display: "flex", alignItems: "center",
          gap: 10, flexShrink: 0,
        }}>
          <GitBranch size={13} color="#475569" style={{ flexShrink: 0 }} />
          <span style={{
            fontSize: 12, color: "#64748b", flex: 1, minWidth: 0,
            overflow: "hidden", textOverflow: "ellipsis", whiteSpace: "nowrap",
          }}>
            {selectedRosRun
              ? `ros2 run ${selectedRosRun.package} ${selectedRosRun.name}`
              : activeEntry ? activeEntry.filePath.split("/").pop()
              : "Select an entry point"}
          </span>

          <span style={{ fontSize: 11, color: "#475569", flexShrink: 0 }}>Depth:</span>
          <div style={{ display: "flex", gap: 3 }}>
            <button
              title="Step back one level"
              onClick={() => setLaunchDepthLimit(Math.max(1, launchDepthLimit - 1))}
              disabled={launchDepthLimit <= 1}
              style={stepBtn(false, launchDepthLimit <= 1)}
            >&larr;</button>
            {[1, 2, 3, 4, 5].map((d) => (
              <button
                key={d}
                title={`Show ${d} level${d !== 1 ? "s" : ""} of includes`}
                onClick={() => setLaunchDepthLimit(d)}
                style={stepBtn(launchDepthLimit === d)}
              >{d}</button>
            ))}
            <button
              title="Show all levels"
              onClick={() => setLaunchDepthLimit(99)}
              style={stepBtn(launchDepthLimit === 99)}
            >&infin;</button>
            <button
              title="Step into one more level"
              onClick={() => setLaunchDepthLimit(Math.min(99, launchDepthLimit + 1))}
              disabled={launchDepthLimit >= 99}
              style={stepBtn(false, launchDepthLimit >= 99)}
            >&rarr;</button>
          </div>
        </div>

        {/* Tree / ros2-run placeholder */}
        <div style={{ flex: 1, overflowY: "auto", padding: "16px 16px 32px" }}>
          {selectedRosRun ? (
            <div style={{ color: "#475569", fontSize: 13 }}>
              Select a launch file from the sidebar to view its include tree.
            </div>
          ) : activeEntry ? (
            <TreeNode
              key={`${launchDepthLimit}-${activeEntry.id}`}
              launch={activeEntry}
              allLaunches={data.launches}
              depth={0}
              maxDepth={launchDepthLimit}
              isLast
              selectedId={selectedLaunchFile}
              onSelect={setSelectedLaunchFile}
              category={categorizeLaunch(activeEntry)}
            />
          ) : (
            <div style={{ color: "#475569", fontSize: 13 }}>
              Select an entry point from the sidebar.
            </div>
          )}
        </div>
      </div>

      {/* ── Detail panel (launch file or ros2 run) ─────────────────────── */}
      {!selectedRosRun && selectedFile && (
        <DetailPanel
          launch={selectedFile}
          entryLaunch={activeEntry}
          allLaunches={data.launches}
        />
      )}
      {selectedRosRun && <RunDetailPanel ep={selectedRosRun} />}
    </div>
  );
}
