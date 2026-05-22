import { useState, useCallback } from "react";
import { Upload, FolderOpen, RefreshCw, CheckCircle, AlertCircle } from "lucide-react";
import { useStore } from "../store";
import { mockData } from "../mockData";
import type { WorkspaceData } from "../types";

export function WorkspaceLoader() {
  const { setData, setLoading, setError, loading, error, data } = useStore();
  const [scanPath, setScanPath] = useState("");

  const loadMock = useCallback(() => {
    setData(mockData);
  }, [setData]);

  const loadFromApi = useCallback(async () => {
    setLoading(true);
    setError(null);
    try {
      const url = scanPath.trim()
        ? `/api/scan?path=${encodeURIComponent(scanPath.trim())}`
        : `/api/scan`;
      const res = await fetch(url);
      if (!res.ok) throw new Error(`Server responded ${res.status}`);
      const json: WorkspaceData = await res.json();
      setData(json);
    } catch (e) {
      setError(String(e));
    } finally {
      setLoading(false);
    }
  }, [scanPath, setData, setLoading, setError]);

  const loadFromFile = useCallback((e: React.ChangeEvent<HTMLInputElement>) => {
    const file = e.target.files?.[0];
    if (!file) return;
    const reader = new FileReader();
    reader.onload = (ev) => {
      try {
        const json: WorkspaceData = JSON.parse(ev.target?.result as string);
        setData(json);
      } catch {
        setError("Invalid JSON file");
      }
    };
    reader.readAsText(file);
  }, [setData, setError]);

  return (
    <div
      style={{
        display: "flex",
        alignItems: "center",
        gap: 12,
        padding: "6px 16px",
        background: "#0a0f1a",
        borderBottom: "1px solid #1e293b",
        flexShrink: 0,
        flexWrap: "wrap",
      }}
    >
      {/* Status */}
      {data && (
        <span style={{ display: "flex", alignItems: "center", gap: 5, fontSize: 11, color: "#34d399" }}>
          <CheckCircle size={12} />
          {data.packages.length} pkgs · {data.nodes.length} nodes · {data.launches.length} launches
          <span style={{ color: "#475569", marginLeft: 4 }}>{data.root}</span>
        </span>
      )}
      {error && (
        <span style={{ display: "flex", alignItems: "center", gap: 5, fontSize: 11, color: "#f87171" }}>
          <AlertCircle size={12} />
          {error}
        </span>
      )}

      <div style={{ flex: 1 }} />

      {/* Path input for scanner */}
      <div style={{ display: "flex", alignItems: "center", gap: 6, background: "#1e293b", borderRadius: 6, padding: "3px 10px", border: "1px solid #334155" }}>
        <FolderOpen size={12} color="#475569" />
        <input
          value={scanPath}
          onChange={(e) => setScanPath(e.target.value)}
          placeholder="Workspace path (empty = scanner default)…"
          style={{ background: "none", border: "none", outline: "none", color: "#e2e8f0", fontSize: 11, width: 300 }}
        />
      </div>

      <button
        onClick={loadFromApi}
        disabled={loading}
        style={{
          display: "flex", alignItems: "center", gap: 5,
          padding: "4px 12px", borderRadius: 6, border: "1px solid #334155",
          background: "#1e293b", color: "#94a3b8", cursor: "pointer", fontSize: 12,
        }}
      >
        <RefreshCw size={12} className={loading ? "spinning" : ""} />
        {loading ? "Scanning…" : "Scan"}
      </button>

      {/* File upload */}
      <label
        style={{
          display: "flex", alignItems: "center", gap: 5,
          padding: "4px 12px", borderRadius: 6, border: "1px solid #334155",
          background: "#1e293b", color: "#94a3b8", cursor: "pointer", fontSize: 12,
        }}
      >
        <Upload size={12} />
        Load JSON
        <input type="file" accept=".json" onChange={loadFromFile} style={{ display: "none" }} />
      </label>

      {/* Demo */}
      <button
        onClick={loadMock}
        style={{
          display: "flex", alignItems: "center", gap: 5,
          padding: "4px 12px", borderRadius: 6, border: "1px solid #1e3a5f",
          background: "#0d2a4a", color: "#60a5fa", cursor: "pointer", fontSize: 12, fontWeight: 600,
        }}
      >
        Demo Data
      </button>
    </div>
  );
}
