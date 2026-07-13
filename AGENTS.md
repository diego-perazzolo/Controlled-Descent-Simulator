# AGENTS.md — guidance for AI-assisted development and review

Read `Readme.md` first for architecture, features and build details. This file
only contains what cannot be inferred from the code: conventions, invariants,
verification commands and the review procedure.

## Golden rules (violating these causes real damage)

1. **Everything under `modeling/notebooks/exported_cpp/` is generated code.**
   Never hand-edit it. To change it, modify the codegen
   (`base_codegen.py`, `rocket_codegen.py`, `quad_codegen.py`) or the notebook
   that drives it, then re-run the notebook's export section.
2. **Error convention: functions returning `bool` return `true` on error.**
   A `return true;` after an error comment is correct — do not "fix" it.
3. **Layer separation:** `ext_*` types live only in `core/ext/` and the
   bindings; `core_*` types never cross into the frontend; the frontend talks
   exclusively through the embind API. Structs at the ext boundary are POD —
   no STL containers in bound structs.
4. **Codegen ↔ C++ coupling:** `state_enum_names`, `param_*` tuples and the
   dims in the `*_codegen.py` configs must match the generated C++ enums and
   the `core_defs.hpp` / `ext_defs.hpp` structs. If you change one side,
   change the other and regenerate.
5. **Debug blocks:** disabled debug logging is wrapped in `#if 0 ... #endif`.
   Do not delete these blocks; do not re-enable them in commits.
6. **Never commit build artifacts** (WASM output, compiled `driver` /
   `_driver` test binaries). They are gitignored — keep it that way.
7. Jupyter notebooks are committed with outputs and execution counts cleared.

## Naming

- Prefixes by layer: `core_` (core C-style API), `ext_` (communication layer).
- C-style boundary APIs: `snake_case` functions, `*_t` suffixed typedefs.
- C++ classes: `PascalCase` types and methods, `m_` prefixed members.
- Python: PEP 8.

## Verification commands

Run from the repo root. Prefer these over inventing new ones.

```bash
# Full WASM build (requires emsdk in the environment)
emcmake cmake -S core -B build -DCMAKE_BUILD_TYPE=Debug && cmake --build build

# Fast C++ syntax check without emsdk (per file)
cd core && clang++ -std=c++20 -fsyntax-only \
  -I. -Iext -IModels -ITrajectory \
  -I../modeling/notebooks/exported_cpp/ROCKET_FF_LQR_01 \
  -I../modeling/notebooks/exported_cpp/QUADROTOR_FF_LQR_01 <file.cpp>

# Frontend syntax check (main.js is an ES module)
cp frontend/main.js /tmp/main_check.mjs && node --check /tmp/main_check.mjs

# Python codegen sanity
python3 -c "import ast; [ast.parse(open('modeling/notebooks/'+f).read()) \
  for f in ('base_codegen.py','rocket_codegen.py','quad_codegen.py')]"

# Notebook JSON validity
python3 -c "import json; json.load(open('modeling/notebooks/<nb>.ipynb'))"
```

## Review procedure

1. Scope: `git diff origin/main..HEAD --stat`, then review only what changed.
2. Run the verification commands above on the touched files.
3. Per area — core C++, `ext/` + bindings, frontend, `modeling/` — check:
   - typos and broken references (comments citing renamed/removed symbols,
     file headers left as `<filename.cpp>` placeholders);
   - copy-paste errors between Rocket and QuadRotor code paths (labels,
     units, comments saying "rocket" in quadrotor context and vice versa);
   - bindings ↔ JS consistency: every function/struct field used in
     `frontend/main.js` exists in `core/ext/bindings.cpp` and vice versa;
   - codegen coupling (golden rule 4).
4. Verify nothing under `exported_cpp/` was hand-edited (golden rule 1).
5. Report findings as: `[SEVERITY] file:line — problem — suggested fix`.
   Severities: HIGH (bugs, UB, broken build), MEDIUM (stale/incorrect docs
   or comments), LOW (typos, style).

## Documentation

- `Readme.md` must be updated when the public API, the models or the repo
  layout change.
- The ASCII architecture diagram uses Unicode box-drawing characters with a
  uniform total line width of 59 — verify alignment after any edit.
- UI-visible strings live in `frontend/index.html` / `main.js`; check their
  English when touched.
