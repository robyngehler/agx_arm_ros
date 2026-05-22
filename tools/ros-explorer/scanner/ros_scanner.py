#!/usr/bin/env python3
"""
ros_scanner.py – Scan a ROS2 workspace and emit a WorkspaceData JSON blob.

Usage:
    python3 ros_scanner.py /path/to/ros_ws          # print JSON to stdout
    python3 ros_scanner.py /path/to/ros_ws -o out.json
    python3 ros_scanner.py /path/to/ros_ws --serve  # HTTP server on :7357

The scanner works entirely via static analysis (AST + regex), so no ROS2
installation is required on the machine running the scanner.
"""

import argparse
import ast
import json
import os
import re
import sys
from datetime import datetime, timezone
from pathlib import Path
from typing import Any


# ── helpers ───────────────────────────────────────────────────────────────────

def _read(path: Path) -> str:
    try:
        return path.read_text(errors="replace")
    except OSError:
        return ""


def _find_files(root: Path, *exts: str) -> list[Path]:
    result: list[Path] = []
    for dirpath, dirnames, filenames in os.walk(root):
        # skip build/install/hidden directories
        dirnames[:] = [
            d for d in dirnames
            if d not in ("build", "install", ".git", "__pycache__", "node_modules")
            and not d.startswith(".")
        ]
        for f in filenames:
            if any(f.endswith(ext) for ext in exts):
                result.append(Path(dirpath) / f)
    return result


# ── package.xml parser ───────────────────────────────────────────────────────

def _parse_package_xml(path: Path) -> dict[str, Any] | None:
    text = _read(path)
    name_m = re.search(r"<name>\s*(.*?)\s*</name>", text)
    if not name_m:
        return None
    name = name_m.group(1)

    build_type = "unknown"
    bt_m = re.search(r"<build_type>\s*(.*?)\s*</build_type>", text)
    if bt_m:
        raw = bt_m.group(1)
        if "ament_python" in raw:
            build_type = "ament_python"
        elif "ament_cmake" in raw:
            build_type = "ament_cmake"
        elif "cmake" in raw:
            build_type = "cmake"

    deps: list[str] = []
    for tag in ("depend", "exec_depend", "build_depend"):
        for m in re.finditer(rf"<{tag}>\s*(.*?)\s*</{tag}>", text):
            deps.append(m.group(1))

    return {
        "name": name,
        "path": str(path.parent.relative_to(path.parent.parent.parent)),
        "deps": list(dict.fromkeys(deps)),
        "buildType": build_type,
    }


# ── msg / srv / action parser ─────────────────────────────────────────────────

def _parse_msg_file(path: Path, pkg: str) -> dict[str, Any]:
    lines = _read(path).splitlines()
    fields = []
    for line in lines:
        line = line.strip()
        if not line or line.startswith("#"):
            continue
        if line == "---":
            break
        parts = line.split()
        if len(parts) >= 2:
            fields.append({"type": parts[0], "name": parts[1]})

    kind_map = {".msg": "msg", ".srv": "srv", ".action": "action"}
    return {
        "name": path.stem,
        "package": pkg,
        "kind": kind_map.get(path.suffix, "msg"),
        "fields": fields,
        "filePath": str(path),
    }


# ── Python node parser ────────────────────────────────────────────────────────

class _NodeVisitor(ast.NodeVisitor):
    """Extract ROS2 pub/sub/service/action/param calls from a Python node."""

    def __init__(self) -> None:
        self.node_name: str | None = None
        self.topics: list[dict] = []
        self.services: list[dict] = []
        self.actions: list[dict] = []
        self.parameters: list[dict] = []
        self.lifecycle = False
        self._class_bases: list[str] = []
        # Variable resolution: declared parameter string defaults, self.attrs, local vars
        self._param_defaults: dict[str, str] = {}
        self._self_attrs: dict[str, str] = {}
        self._locals: dict[str, str] = {}

    # ── variable resolution helpers ───────────────────────────────────────────

    def _resolve(self, n: ast.expr | None) -> str:
        """Resolve an AST expression to a string using known variable bindings."""
        if n is None:
            return ""
        if isinstance(n, ast.Constant):
            return str(n.value)
        # self.attr → look up in tracked self-attributes
        if (isinstance(n, ast.Attribute) and isinstance(n.value, ast.Name)
                and n.value.id == "self"):
            val = self._self_attrs.get(n.attr, "")
            if val:
                return val
        # local variable → look up in tracked locals
        if isinstance(n, ast.Name):
            val = self._locals.get(n.id, "")
            if val:
                return val
        # fallback: return source representation
        if hasattr(ast, "unparse"):
            return ast.unparse(n)
        return ""

    def _extract_string_value(self, node: ast.expr) -> str:
        """Extract a plain string from an assignment RHS.

        Handles: "literal", str(expr), self.get_parameter("name").value
        """
        if isinstance(node, ast.Constant) and isinstance(node.value, str):
            return node.value
        # str(...) wrapper
        if (isinstance(node, ast.Call) and isinstance(node.func, ast.Name)
                and node.func.id == "str" and node.args):
            return self._extract_string_value(node.args[0])
        # expr.value  →  (self.)get_parameter("name").value
        if (isinstance(node, ast.Attribute) and node.attr == "value"
                and isinstance(node.value, ast.Call)):
            call = node.value
            if (isinstance(call.func, ast.Attribute)
                    and call.func.attr == "get_parameter" and call.args):
                pname_node = call.args[0]
                if (isinstance(pname_node, ast.Constant)
                        and isinstance(pname_node.value, str)):
                    return self._param_defaults.get(pname_node.value, "")
        return ""

    # ── assignment tracking ───────────────────────────────────────────────────

    def visit_Assign(self, node: ast.Assign) -> None:
        """Track self.attr = <string> and local_var = <string> assignments."""
        val = self._extract_string_value(node.value)
        if val:
            for target in node.targets:
                if isinstance(target, ast.Name):
                    self._locals[target.id] = val
                elif (isinstance(target, ast.Attribute)
                        and isinstance(target.value, ast.Name)
                        and target.value.id == "self"):
                    self._self_attrs[target.attr] = val
        self.generic_visit(node)

    # ── class definitions ─────────────────────────────────────────────────────

    def visit_ClassDef(self, node: ast.ClassDef) -> None:
        for base in node.bases:
            name = ast.unparse(base) if hasattr(ast, "unparse") else ""
            if "LifecycleNode" in name:
                self.lifecycle = True
            if "Node" in name:
                self._class_bases.append(name)
        self.generic_visit(node)

    # ── ROS2 API call extraction ──────────────────────────────────────────────

    def visit_Call(self, node: ast.Call) -> None:  # noqa: C901
        func = node.func
        fname = ""
        if isinstance(func, ast.Attribute):
            fname = func.attr
        elif isinstance(func, ast.Name):
            fname = func.id

        # super().__init__("node_name")
        if fname == "__init__" and node.args:
            val = self._resolve(node.args[0])
            if val and not val.startswith("("):
                self.node_name = val

        # create_publisher(MsgType, "topic", qos)
        if fname == "create_publisher" and len(node.args) >= 2:
            msg_type = self._resolve(node.args[0])
            topic = self._resolve(node.args[1])
            if topic and "." not in topic and not topic.startswith("("):
                self.topics.append({
                    "topic": topic,
                    "msgType": msg_type.replace(".", "/"),
                    "direction": "pub",
                })

        # create_subscription(MsgType, "topic", cb, qos)
        if fname == "create_subscription" and len(node.args) >= 2:
            msg_type = self._resolve(node.args[0])
            topic = self._resolve(node.args[1])
            if topic and "." not in topic and not topic.startswith("("):
                self.topics.append({
                    "topic": topic,
                    "msgType": msg_type.replace(".", "/"),
                    "direction": "sub",
                })

        # create_service(SrvType, "name", cb)
        if fname == "create_service" and len(node.args) >= 2:
            srv_type = self._resolve(node.args[0])
            svc = self._resolve(node.args[1])
            if svc and "." not in svc and not svc.startswith("("):
                self.services.append({
                    "service": svc,
                    "srvType": srv_type.replace(".", "/"),
                    "role": "server",
                })

        # create_client(SrvType, "name")
        if fname == "create_client" and len(node.args) >= 2:
            srv_type = self._resolve(node.args[0])
            svc = self._resolve(node.args[1])
            if svc and "." not in svc and not svc.startswith("("):
                self.services.append({
                    "service": svc,
                    "srvType": srv_type.replace(".", "/"),
                    "role": "client",
                })

        # ActionServer(node_ref, ActionType, "action_name", execute_callback=...)
        if fname == "ActionServer" and len(node.args) >= 3:
            action_type = self._resolve(node.args[1])
            action = self._resolve(node.args[2])
            if action and not action.startswith("("):
                self.actions.append({
                    "action": action,
                    "actionType": action_type.replace(".", "/"),
                    "role": "server",
                })

        # ActionClient(node_ref, ActionType, "action_name")
        if fname == "ActionClient" and len(node.args) >= 3:
            action_type = self._resolve(node.args[1])
            action = self._resolve(node.args[2])
            if action and not action.startswith("("):
                self.actions.append({
                    "action": action,
                    "actionType": action_type.replace(".", "/"),
                    "role": "client",
                })

        # declare_parameter("name", default, ParameterDescriptor(...))
        if fname == "declare_parameter" and node.args:
            pname = self._resolve(node.args[0])
            default: str | None = None
            if len(node.args) > 1:
                default_node = node.args[1]
                if (isinstance(default_node, ast.Constant)
                        and isinstance(default_node.value, str)):
                    default = default_node.value
                    # Cache string defaults for later variable resolution
                    self._param_defaults[pname] = default
            desc = ""
            for kw in node.keywords:
                if kw.arg == "description":
                    desc = self._resolve(kw.value)
            if pname:
                self.parameters.append({
                    "name": pname,
                    "type": "unknown",
                    "default": default,
                    "description": desc,
                })

        self.generic_visit(node)


def _parse_python_node(path: Path, pkg: str) -> dict[str, Any] | None:
    text = _read(path)
    if "rclpy" not in text and "Node" not in text:
        return None
    try:
        tree = ast.parse(text)
    except SyntaxError:
        return None

    visitor = _NodeVisitor()
    visitor.visit(tree)

    if not visitor.topics and not visitor.services and not visitor.actions and not visitor.node_name:
        return None

    node_name = visitor.node_name or path.stem
    node_id = f"{pkg}/{node_name}"

    return {
        "id": node_id,
        "nodeName": node_name,
        "package": pkg,
        "filePath": str(path),
        "topics": visitor.topics,
        "services": visitor.services,
        "actions": visitor.actions,
        "parameters": visitor.parameters,
        "lifecycleNode": visitor.lifecycle,
        "lifecycleStates": [],
    }


# ── launch file parser ────────────────────────────────────────────────────────

class _LaunchVisitor(ast.NodeVisitor):
    def __init__(self) -> None:
        self.args: list[dict] = []
        self.nodes: list[dict] = []
        self.includes: list[dict] = []

    def _str(self, n: ast.expr | None) -> str:
        if n is None:
            return ""
        if isinstance(n, ast.Constant):
            return str(n.value)
        if hasattr(ast, "unparse"):
            return ast.unparse(n)
        return ""

    def _extract_launch_filename(self, n: ast.expr) -> str:
        """Best-effort extraction of a launch filename from a path expression.

        Handles common patterns:
          os.path.join(get_package_share_directory('pkg'), 'launch', 'file.launch.py')
          PathJoinSubstitution([FindPackageShare('pkg'), 'launch', 'file.launch.py'])
        Falls back to ast.unparse for dynamic/variable expressions.
        """
        # Direct string constant
        if isinstance(n, ast.Constant) and isinstance(n.value, str):
            return n.value
        # os.path.join(..., 'file.launch.py') — last positional arg is the filename
        if isinstance(n, ast.Call):
            func = n.func
            is_join = isinstance(func, ast.Attribute) and func.attr == "join"
            if is_join and n.args:
                last = n.args[-1]
                if isinstance(last, ast.Constant) and isinstance(last.value, str):
                    return last.value
        # PathJoinSubstitution([..., 'file.launch.py']) — list/tuple, last element
        if isinstance(n, (ast.List, ast.Tuple)) and n.elts:
            last = n.elts[-1]
            if isinstance(last, ast.Constant) and isinstance(last.value, str):
                return last.value
        # Walk all constants and return the last one ending in .launch.py / .launch
        for child in ast.walk(n):
            if isinstance(child, ast.Constant) and isinstance(child.value, str):
                if child.value.endswith(".launch.py") or child.value.endswith(".launch"):
                    return child.value
        # Give up — return the full unparse so the frontend can still display something
        if hasattr(ast, "unparse"):
            return ast.unparse(n)
        return ""

    def _kw(self, call: ast.Call, key: str) -> str:
        for kw in call.keywords:
            if kw.arg == key:
                return self._str(kw.value)
        return ""

    def visit_Call(self, node: ast.Call) -> None:  # noqa: C901
        func = node.func
        fname = ""
        if isinstance(func, ast.Attribute):
            fname = func.attr
        elif isinstance(func, ast.Name):
            fname = func.id

        # DeclareLaunchArgument(name, default_value=..., description=..., choices=...)
        if fname == "DeclareLaunchArgument" and node.args:
            name = self._str(node.args[0])
            default = self._kw(node, "default_value")
            desc = self._kw(node, "description")
            choices_node = next(
                (kw.value for kw in node.keywords if kw.arg == "choices"), None
            )
            choices: list[str] | None = None
            if isinstance(choices_node, (ast.List, ast.Tuple)):
                choices = [self._str(e) for e in choices_node.elts]
            if name:
                self.args.append({
                    "name": name,
                    "default": default or None,
                    "description": desc or None,
                    "choices": choices,
                })

        # Node(package=..., executable=..., name=..., namespace=...)
        if fname == "Node":
            pkg = self._kw(node, "package")
            exe = self._kw(node, "executable")
            nm = self._kw(node, "name") or None
            ns = self._kw(node, "namespace") or None
            cond = self._kw(node, "condition") or None
            if pkg and exe:
                self.nodes.append({
                    "package": pkg,
                    "executable": exe,
                    "name": nm,
                    "namespace": ns,
                    "condition": cond,
                })

        # IncludeLaunchDescription(PythonLaunchDescriptionSource(...))
        if fname == "IncludeLaunchDescription" and node.args:
            src = node.args[0]
            file_str = ""
            if isinstance(src, ast.Call) and src.args:
                # src = PythonLaunchDescriptionSource(path_expr)
                # src.args[0] is the path expression — extract filename from it
                file_str = self._extract_launch_filename(src.args[0])
            if not file_str:
                file_str = self._str(src)
            cond = self._kw(node, "condition") or None
            if file_str:
                self.includes.append({"file": file_str, "condition": cond})

        self.generic_visit(node)


def _parse_launch_file(path: Path, pkg: str, rel: Path) -> dict[str, Any] | None:
    text = _read(path)
    if "generate_launch_description" not in text:
        return None
    try:
        tree = ast.parse(text)
    except SyntaxError:
        return None

    visitor = _LaunchVisitor()
    visitor.visit(tree)

    launch_id = f"{pkg}/launch/{path.stem}"
    return {
        "id": launch_id,
        "filePath": str(rel),
        "package": pkg,
        "args": visitor.args,
        "nodes": visitor.nodes,
        "includes": visitor.includes,
    }


# ── setup.py entry-point scanner ─────────────────────────────────────────────

_CONSOLE_SCRIPT_RE = re.compile(
    r"""['"]([\w\-]+)\s*=\s*([\w\.]+):([\w]+)['"]"""
)


def _parse_setup_entry_points(setup_py: Path, pkg_name: str) -> list[dict[str, Any]]:
    """Extract console_scripts from setup.py as ros2-run entry points."""
    text = _read(setup_py)
    # Only look inside the console_scripts block
    cs_match = re.search(r"console_scripts\s*[:=]\s*\[([^\]]*)\]", text, re.DOTALL)
    if not cs_match:
        return []
    block = cs_match.group(1)
    results = []
    for m in _CONSOLE_SCRIPT_RE.finditer(block):
        name, module, fn = m.group(1), m.group(2), m.group(3)
        results.append({
            "name": name,
            "module": f"{module}:{fn}",
            "package": pkg_name,
        })
    return results


# ── workspace scanner ─────────────────────────────────────────────────────────

def scan_workspace(root: str) -> dict[str, Any]:
    ws = Path(root).resolve()
    src = ws / "src"
    if not src.exists():
        src = ws  # flat workspace

    packages: list[dict] = []
    nodes: list[dict] = []
    launches: list[dict] = []
    messages: list[dict] = []
    entry_points: list[dict] = []

    for pkg_xml in _find_files(src, "package.xml"):
        pkg_info = _parse_package_xml(pkg_xml)
        if not pkg_info:
            continue
        packages.append(pkg_info)
        pkg_name = pkg_info["name"]
        pkg_dir = pkg_xml.parent

        # messages
        for ext in (".msg", ".srv", ".action"):
            for f in _find_files(pkg_dir, ext):
                messages.append(_parse_msg_file(f, pkg_name))

        # python nodes
        for f in _find_files(pkg_dir, ".py"):
            if "launch" in str(f):
                continue
            result = _parse_python_node(f, pkg_name)
            if result:
                nodes.append(result)

        # launch files
        for f in _find_files(pkg_dir, ".launch.py"):
            rel = f.relative_to(ws) if f.is_relative_to(ws) else f
            result = _parse_launch_file(f, pkg_name, rel)
            if result:
                launches.append(result)

        # ros2 run entry points from setup.py console_scripts
        setup_py = pkg_dir / "setup.py"
        if setup_py.exists():
            for ep in _parse_setup_entry_points(setup_py, pkg_name):
                entry_points.append(ep)

    return {
        "root": str(ws),
        "scannedAt": datetime.now(timezone.utc).isoformat(),
        "packages": packages,
        "nodes": nodes,
        "launches": launches,
        "messages": messages,
        "entryPoints": entry_points,
    }


# ── HTTP server mode ──────────────────────────────────────────────────────────

def _serve(workspace: str, port: int = 7357) -> None:
    from http.server import BaseHTTPRequestHandler, HTTPServer
    from urllib.parse import parse_qs, urlparse
    import socket

    frontend_dist = Path(__file__).parent.parent / "dist"

    class _ReusableHTTPServer(HTTPServer):
        allow_reuse_address = True
        def server_bind(self):
            self.socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
            super().server_bind()

    class Handler(BaseHTTPRequestHandler):
        def log_message(self, fmt: str, *args: Any) -> None:
            pass  # suppress default access log

        def _send_json(self, data: Any, status: int = 200) -> None:
            body = json.dumps(data).encode()
            self.send_response(status)
            self.send_header("Content-Type", "application/json")
            self.send_header("Access-Control-Allow-Origin", "*")
            self.send_header("Content-Length", str(len(body)))
            self.end_headers()
            self.wfile.write(body)

        def _send_file(self, path: Path) -> None:
            ext_map = {
                ".html": "text/html",
                ".js": "application/javascript",
                ".css": "text/css",
                ".svg": "image/svg+xml",
                ".ico": "image/x-icon",
                ".json": "application/json",
                ".woff2": "font/woff2",
            }
            ct = ext_map.get(path.suffix, "application/octet-stream")
            data = path.read_bytes()
            self.send_response(200)
            self.send_header("Content-Type", ct)
            self.send_header("Content-Length", str(len(data)))
            self.end_headers()
            self.wfile.write(data)

        def do_GET(self) -> None:
            parsed = urlparse(self.path)
            path = parsed.path
            qs = parse_qs(parsed.query)

            if path == "/api/scan":
                ws_path = qs.get("path", [workspace])[0]
                try:
                    data = scan_workspace(ws_path)
                    self._send_json(data)
                except Exception as e:
                    self._send_json({"error": str(e)}, 500)
                return

            # Static file serving from dist/
            if frontend_dist.exists():
                static_path = (frontend_dist / path.lstrip("/")).resolve()
                # Security: ensure path stays within dist
                if not str(static_path).startswith(str(frontend_dist)):
                    self.send_response(403)
                    self.end_headers()
                    return
                if static_path.is_file():
                    self._send_file(static_path)
                    return
                # SPA fallback
                index = frontend_dist / "index.html"
                if index.exists():
                    self._send_file(index)
                    return

            self.send_response(404)
            self.end_headers()

    print(f"ROS Explorer scanner serving on http://localhost:{port}")
    print(f"  API: http://localhost:{port}/api/scan?path={workspace}")
    if frontend_dist.exists():
        print(f"  UI:  http://localhost:{port}/")
    _ReusableHTTPServer(("", port), Handler).serve_forever()


# ── main ──────────────────────────────────────────────────────────────────────

def main() -> None:
    parser = argparse.ArgumentParser(description="ROS2 workspace static scanner")
    parser.add_argument("workspace", nargs="?", default=".", help="Path to ROS2 workspace root")
    parser.add_argument("-o", "--output", help="Write JSON to file instead of stdout")
    parser.add_argument("--serve", action="store_true", help="Start HTTP server (API + UI)")
    parser.add_argument("--port", type=int, default=7357, help="HTTP server port (default: 7357)")
    args = parser.parse_args()

    if args.serve:
        _serve(args.workspace, args.port)
        return

    data = scan_workspace(args.workspace)
    out = json.dumps(data, indent=2, ensure_ascii=False)
    if args.output:
        Path(args.output).write_text(out)
        print(f"Written to {args.output}", file=sys.stderr)
    else:
        print(out)


if __name__ == "__main__":
    main()
