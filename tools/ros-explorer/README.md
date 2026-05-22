# ROS Explorer

Interactive browser-based tool for exploring ROS2 workspaces.
Built with Vite + React + TypeScript · zero ROS installation required.

## Quick start

```bash
cd tools/ros-explorer
./start.sh                          # dev mode, scans ../../ (workspace root)
./start.sh /path/to/other/ros_ws    # scan a specific workspace
./start.sh --build                  # production build → served on :7357
```

No scanner? Click **Demo Data** in the UI to load built-in mock data.

## Features

| View | Description |
|---|---|
| **Node Graph** | Interactive graph of all ROS nodes connected via topics/services/actions. Filter by package, connection type, or topic name. |
| **Launch Trees** | Hierarchical view of launch files, arguments, sub-launches, and spawned nodes. Expandable depth (default 3). |
| **Lifecycle / States** | ROS2 standard lifecycle diagram, per-node state machines, and topic sequence flows via Mermaid. |

## Scanner

`scanner/ros_scanner.py` – pure Python (≥3.10) static analyser.
No ROS or colcon required.

```bash
# HTTP API + UI (after npm run build)
python3 scanner/ros_scanner.py /ros_ws --serve

# Offline JSON snapshot
python3 scanner/ros_scanner.py /ros_ws -o workspace.json
```
h HMR and some ESLint rules.

Currently, two official plugins are available:

- [@vitejs/plugin-react](https://github.com/vitejs/vite-plugin-react/blob/main/packages/plugin-react) uses [Oxc](https://oxc.rs)
- [@vitejs/plugin-react-swc](https://github.com/vitejs/vite-plugin-react/blob/main/packages/plugin-react-swc) uses [SWC](https://swc.rs/)

## React Compiler

The React Compiler is not enabled on this template because of its impact on dev & build performances. To add it, see [this documentation](https://react.dev/learn/react-compiler/installation).

## Expanding the ESLint configuration

If you are developing a production application, we recommend updating the configuration to enable type-aware lint rules:

```js
export default defineConfig([
  globalIgnores(['dist']),
  {
    files: ['**/*.{ts,tsx}'],
    extends: [
      // Other configs...

      // Remove tseslint.configs.recommended and replace with this
      tseslint.configs.recommendedTypeChecked,
      // Alternatively, use this for stricter rules
      tseslint.configs.strictTypeChecked,
      // Optionally, add this for stylistic rules
      tseslint.configs.stylisticTypeChecked,

      // Other configs...
    ],
    languageOptions: {
      parserOptions: {
        project: ['./tsconfig.node.json', './tsconfig.app.json'],
        tsconfigRootDir: import.meta.dirname,
      },
      // other options...
    },
  },
])
```

You can also install [eslint-plugin-react-x](https://github.com/Rel1cx/eslint-react/tree/main/packages/plugins/eslint-plugin-react-x) and [eslint-plugin-react-dom](https://github.com/Rel1cx/eslint-react/tree/main/packages/plugins/eslint-plugin-react-dom) for React-specific lint rules:

```js
// eslint.config.js
import reactX from 'eslint-plugin-react-x'
import reactDom from 'eslint-plugin-react-dom'

export default defineConfig([
  globalIgnores(['dist']),
  {
    files: ['**/*.{ts,tsx}'],
    extends: [
      // Other configs...
      // Enable lint rules for React
      reactX.configs['recommended-typescript'],
      // Enable lint rules for React DOM
      reactDom.configs.recommended,
    ],
    languageOptions: {
      parserOptions: {
        project: ['./tsconfig.node.json', './tsconfig.app.json'],
        tsconfigRootDir: import.meta.dirname,
      },
      // other options...
    },
  },
])
```
