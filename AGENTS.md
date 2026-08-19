# AGENTS.md — guidance for AI-assisted development and review

Read `Readme.md` first for architecture and features; build details and the
full repo layout are in `docs/build.md`, the ext API reference in
`docs/api.md`, the ArduPilot SITL run guide in `docs/sitl.md`. This file only
contains what cannot be inferred from the code:
conventions, invariants, verification commands and the review procedure.

## Golden rules (violating these causes real damage)

1. **Every `exported_cpp/` folder is generated code — never hand-edit it.**
   - `modeling/notebooks/exported_cpp/` — to change it, modify the codegen
     (`base_codegen.py`, `rocket_codegen.py`, `quad_codegen.py`) or the
     notebook that drives it, then re-run the notebook's export section;
   - `apps/common/exported_cpp/` and `apps/ws-served/exported_cpp/` — to
     change them, edit `apps/common/ext_api.py` and run
     `python3 apps/common/gen_ext.py` (the app builds also re-run it
     automatically when the description changes).
   - `plants/sitl/mavlink/` is **vendored** third-party generated code — same
     rule: never hand-edit it, re-vendor at a new upstream commit per
     `plants/sitl/mavlink/VENDORED.md`. The wire contract is pinned in
     `plants/sitl/mavlink_pin.hpp` (the only file that may include the
     vendored headers); a re-vendor that drifts it must fail the build until
     the pins are reviewed and updated together.
2. **Error convention: functions returning `bool` return `true` on error.**
   A `return true;` after an error comment is correct — do not "fix" it.
3. **Layer separation:** `ext_*` types live only under `apps/` (the `common/`
   boundary and the per-app adapters); `core_*` types never cross into the
   frontend; the frontend talks exclusively through the embind API. Structs
   at the ext boundary are POD — no STL containers in bound structs.
   Dependencies flow one way: `apps/* → apps/common → core → libs/*` and
   `apps/* → libs/*`, never back. `libs/` is protocol- and domain-agnostic
   infrastructure: nothing under `libs/` may include app or core headers.
4. **Codegen ↔ C++ coupling:** `state_enum_names`, `param_*` tuples and the
   dims in the `*_codegen.py` configs must match the generated C++ enums and
   the `core_defs.hpp` / `ext_defs.hpp` structs. If you change one side,
   change the other and regenerate.
5. **Debug blocks:** disabled debug logging is wrapped in `#if 0 ... #endif`.
   Do not delete these blocks; do not re-enable them in commits.
6. **Never commit build artifacts** (WASM output, `cds_server`, compiled
   `driver` / `_driver` test binaries). They are gitignored — keep it that way.
7. Jupyter notebooks are committed with outputs and execution counts cleared.
8. **The ext API is described in `apps/common/ext_api.py`, nowhere else.**
   To add or change a command: edit the description, run
   `python3 apps/common/gen_ext.py` (regenerates structs, contract, bindings
   and the whole ws wire layer, including the exact-size `static_assert`s),
   then implement the adapter function in the hand-written
   `apps/common/ext_comm.cpp`.
   `python3 apps/common/gen_ext.py --check` must pass before every commit.
   Wire-crossing fields must be `ext_coord_t` or `bool` only — never
   `double`, `size_t`, `long` or pointers, whose width/padding is
   architecture-dependent. Request/response correlation is owned by the
   `libs/ws` transport (4-byte id framing) — never add ids to the protocol.
   The protocol version byte is a generator-computed fingerprint of the
   description — never hand-set or "fix" it.
   Command ids stay contiguous — renumber the survivors when removing one.
   Never hardcode ids or wire sizes in tests: parse them from the generated
   `ws_protocol.hpp`, as `test_protocol.py` does.
9. **Concurrency and tick semantics:** shared simulation state (model, plant,
   trajectory) lives behind the SystemManager mutex — every public entry
   point takes the lock. The tick path never blocks: no I/O, no unbounded
   waits; the plant exchange goes exclusively through the wait-free mailboxes
   (`PushCommands` / `PullMeasurements` are non-virtual by design — do not
   bypass or override them). The dt passed to `ExecuteTick` is the *measured*
   wall-clock elapsed, clamped by the tick generator: simulation time is
   wall-anchored by design — do not replace it with the nominal tick period,
   and do not reintroduce integer `duration_cast` in the elapsed measurement
   (sub-unit iterations truncate to zero and silently freeze the simulation).
10. **Controllers are generic C++ plus a Python conformance certificate.**
    Hand-written control algorithms (iLQR today, LQR next) live in
    `libs/control` as model- and protocol-agnostic, header-only infrastructure —
    parameterised by the model and cost through callables, never carrying
    vehicle-specific code. Every such controller MUST ship a runnable,
    dependency-light **C++↔Python conformance certificate**: a
    `libs/control/bind/` C-ABI shim plus a stdlib-only script that certifies the
    C++ result against an independent Python oracle on a synthetic benchmark
    (for iLQR, that the returned command sequence is a constrained optimum — a
    ~0 box-projected KKT residual). A controller without a green conformance is
    not done. Model-specific pieces (the tracking cost, the reference sampling)
    live with the model under `core/Models`, not in `libs/control`. This mirrors
    the generate-vs-hand-write split: symbolic model artifacts are generated
    (codegen), algorithmic controllers are hand-written in C++ and validated
    from Python.

## Naming

- Prefixes by layer: `core_` (core C-style API), `ext_` (communication layer).
- C-style boundary APIs: `snake_case` functions, `*_t` suffixed typedefs.
- C++ classes: `PascalCase` types and methods, `m_` prefixed members; core
  classes live in `namespace CDS`.
- Every file starts with the standard license/description header block;
  headers use `#pragma once`.
- Members are initialized in the constructor initializer list (declaration
  order), not with in-class default initializers in the `.hpp`.
- Document *behaviour* contracts on functions, *data* layout on structs.
- Python: PEP 8.

## Verification commands

Run from the repo root. Prefer these over inventing new ones.

```bash
# wasm-only app (requires emsdk; output lands in build/)
emcmake cmake -S apps/wasm-only -B build-wasm-only -DCMAKE_BUILD_TYPE=Debug && cmake --build build-wasm-only

# ws-served app: WASM proxy (emsdk; output lands in build/) + native core server
# (always pass a build type: an empty CMAKE_BUILD_TYPE silently builds at -O0)
emcmake cmake -S apps/ws-served/client -B build-ws-client -DCMAKE_BUILD_TYPE=Debug && cmake --build build-ws-client
cmake -S apps/ws-served/server -B build-server -DCMAKE_BUILD_TYPE=Release && cmake --build build-server

# Fast C++ syntax check without emsdk (per file)
clang++ -std=c++20 -fsyntax-only \
  -Icore -Icore/System -Icore/Plant -Icore/Models -Icore/Trajectory \
  -Iapps/common/exported_cpp -Iapps/ws-served/exported_cpp \
  -Iapps/ws-served/server -Ilibs/ws -Ilibs/sync -Ilibs/integrate -Ilibs/control \
  -Ilibs/log -Ilibs/profile -Ilibs/record \
  -Imodeling/notebooks/exported_cpp/ROCKET_FF_LQR_01 \
  -Imodeling/notebooks/exported_cpp/QUADROTOR_FF_LQR_01 \
  -Imodeling/notebooks/exported_cpp/QUADROTOR_MPC_01 \
  -Imodeling/notebooks/exported_cpp/ROCKET_MPC_01 <file.cpp>

# Frontend syntax check (main.js is an ES module)
cp frontend/main.js /tmp/main_check.mjs && node --check /tmp/main_check.mjs

# ext API generator: verify generated files match apps/common/ext_api.py
python3 apps/common/gen_ext.py --check

# ws protocol end-to-end test (builds nothing: needs build-server/cds_server)
python3 apps/ws-served/test/test_protocol.py ./build-server/cds_server

# plant machinery integration tests (native)
cmake -S plants/test -B build-plants-test -DCMAKE_BUILD_TYPE=Release
cmake --build build-plants-test
./build-plants-test/driver        # mailboxes, freshness, lifecycle
./build-plants-test/sitl_driver   # SITL plant vs in-process fake ArduCopter

# generic control-solver tests (native, self-contained: no core, no quaternions)
cmake -S libs/control/test -B build-ilqr-test -DCMAKE_BUILD_TYPE=Release
cmake --build build-ilqr-test
./build-ilqr-test/ilqr_test        # double-integrator: loose-box reach + tight-box feasibility
./build-ilqr-test/lqr_test         # LQR synthesis: known-answer gains + closed-loop stability

# generic estimator tests (native, self-contained: no core, no quaternions)
cmake -S libs/estimate/test -B build-observer-test -DCMAKE_BUILD_TYPE=Release
cmake --build build-observer-test
./build-observer-test/observer_test # observer synthesis (dual LQR): known-answer gains + Step convergence

# logger + profiler tests (native, self-contained: no core, no protocol)
cmake -S libs/log/test -B build-log-test -DCMAKE_BUILD_TYPE=Release
cmake --build build-log-test
./build-log-test/log_test          # deferred formatting + level filter + drop counting
cmake -S libs/profile/test -B build-profile-test -DCMAKE_BUILD_TYPE=Release
cmake --build build-profile-test
./build-profile-test/profile_test  # scope aggregates + wait-free snapshot round-trip
cmake -S libs/record/test -B build-record-test -DCMAKE_BUILD_TYPE=Release
cmake --build build-record-test
./build-record-test/record_test    # wide-CSV round-trip + explicit drop counting

# speed benchmarks (NOT a CI gate — timing-dependent). perf_bench sizes the
# per-call cost of the logger/profiler/recorder macros ON vs OFF and the drain
# (perf_bench_off shows the compile-out residual); model_bench times one
# PerformIntegration per model. `python3 bench/report.py build-bench` assembles
# the Markdown artifact the CI `benchmark` job uploads.
# RULE: every runtime model MUST be represented in model_bench (add new models
# there) so its integration cost is tracked. docs/benchmark.md is the stable,
# hand-curated companion (the report artifact is ephemeral, gitignored).
# RULE: if you run these benchmarks and a figure differs markedly from the one
# quoted in docs/benchmark.md (rule of thumb: a model's tick cost or a diagnostics
# cost moved by roughly an order of magnitude, or clearly regressed beyond run-to-
# run noise), do NOT silently update the doc — flag it to the developer: it may be
# a real performance regression, or benchmark.md may be due for a refresh.
cmake -S bench -B build-bench -DCMAKE_BUILD_TYPE=Release
cmake --build build-bench
./build-bench/perf_bench            # features compiled in (runtime ON/OFF)
./build-bench/perf_bench_off        # features compiled out (macros stripped ~0)
./build-bench/model_bench          # per-model integration cost (mean/p50/p95/max)

# controller C++<->Python conformance (golden rule 10). ctypes, no numpy:
# certifies each hand-written solver against an independent Python oracle on a
# synthetic model. iLQR: the returned command sequence is a constrained optimum
# (box-projected KKT residual ~0). LQR: the returned gain is an optimum, checked
# by a Lyapunov-stationarity residual ||K - R^-1 B' X|| ~0 (X from the Lyapunov
# equation) with X positive-definite as the Hurwitz witness. Both bind libraries
# build from the same CMake (build-ilqr-bind).
cmake -S libs/control/bind -B build-ilqr-bind -DCMAKE_BUILD_TYPE=Release
cmake --build build-ilqr-bind
python3 libs/control/bind/ilqr_conformance.py build-ilqr-bind
python3 libs/control/bind/lqr_conformance.py  build-ilqr-bind

# estimator C++<->Python conformance (golden rule 10, dual of the LQR one). The
# observer gain is synthesised as the LQR dual (L = lqr(A',C',Qw,Rv)') and here
# certified an optimum by the filtering Lyapunov-stationarity residual
# ||L - P C' Rv^-1|| ~0 (P from A_cl P + P A_cl' = -(Qw + L Rv L'), A_cl = A - L C)
# with P positive-definite as the Hurwitz witness.
cmake -S libs/estimate/bind -B build-observer-bind -DCMAKE_BUILD_TYPE=Release
cmake --build build-observer-bind
python3 libs/estimate/bind/observer_conformance.py build-observer-bind

# model tests (native, exercise the runtime models end to end)
cmake -S core/Models/test -B build-mpc-model-test -DCMAKE_BUILD_TYPE=Release
cmake --build build-mpc-model-test
./build-mpc-model-test/mpc_model_test        # QuadRotorMPC: Poly4 tracking + gust rejection
./build-mpc-model-test/rocket_mpc_model_test # RocketMPC: Poly4+yaw tracking + gust rejection
./build-mpc-model-test/lqr_model_test        # Rocket/QuadRotor FF-LQR: runtime gain bridge + Q/R retune

# Python codegen sanity (base_codegen shared at root; model codegens under model/)
python3 -c "import ast; [ast.parse(open(f).read()) for f in ( \
  'modeling/notebooks/base_codegen.py', \
  'modeling/notebooks/model/quad_codegen.py', \
  'modeling/notebooks/model/rocket_codegen.py', \
  'modeling/notebooks/model/mpc_codegen.py')]"

# Notebook JSON validity (notebooks live under model/ or control/)
python3 -c "import json; json.load(open('modeling/notebooks/model/<nb>.ipynb'))"
```

## Review procedure

1. Scope: `git diff origin/main..HEAD --stat`, then review only what changed.
2. Run the verification commands above on the touched files.
3. Per area — core C++, `apps/` (adapters + bindings + server), frontend,
   `modeling/` — check:
   - typos and broken references (comments citing renamed/removed symbols,
     file headers left as `<filename.cpp>` placeholders);
   - copy-paste errors between Rocket and QuadRotor code paths (labels,
     units, comments saying "rocket" in quadrotor context and vice versa);
   - bindings ↔ JS consistency: every function/struct field used in
     `frontend/main.js` exists in `apps/common/bindings.cpp` and vice versa;
   - codegen coupling (golden rule 4) and ws-protocol coupling (golden
     rule 8).
4. Verify nothing under `exported_cpp/` was hand-edited (golden rule 1).
5. Report findings as: `[SEVERITY] file:line — problem — suggested fix`.
   Severities: HIGH (bugs, UB, broken build), MEDIUM (stale/incorrect docs
   or comments), LOW (typos, style).

## Collaboration

- Design questions get discussed before code changes: when asked a question
  or an opinion, propose — do not modify files until explicitly told to
  proceed.
- Do not "normalize" odd-looking tuning values (e.g. a very small
  `TIMESTEP_MIN_S`): they may be deliberate experimentation — ask first.

## Documentation

- `docs/api.md` must be updated when the public ext API changes (functions or
  bound structs); `docs/build.md` when the build commands or the repo layout
  change; `docs/models.md` when a model's state vector, forces or controller
  changes; `Readme.md` when the models or the features change.
- The ASCII architecture diagram in `Readme.md` uses Unicode box-drawing
  characters with a uniform total line width of 59 — verify alignment after
  any edit.
- UI-visible strings live in `frontend/index.html` / `main.js`; check their
  English when touched.
