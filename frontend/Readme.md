# Frontend

## Local development

From the project root:

```bash
python3 tools/serve.py 8080
```

Open: http://localhost:8080/frontend/

The frontend imports `build/simulator.js`: it runs whichever browser app was
built last into `build/` — `apps/wasm-only` (core in the browser) or
`apps/ws-served/client` (thin proxy talking to `cds_server` over WebSocket).
See [docs/build.md](../docs/build.md).

> **Tip:** Use `Cmd + Shift + R` (macOS) or `Ctrl + Shift + R` (Windows/Linux) for a hard refresh after recompiling the WASM module.
