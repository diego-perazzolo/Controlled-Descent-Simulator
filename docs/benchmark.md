# Benchmarks

This page summarises how fast the simulator runs and what the diagnostics cost.

**Where the numbers come from.** Two small native programs under `bench/`:

- `model_bench` — times one physics tick for each vehicle model;
- `perf_bench` — times the logging / profiling / recording calls, on vs off.

The CI **`benchmark` job** runs them on every push and uploads a fresh
**`benchmark-report`** artifact (download it from the run's summary page). To
run them yourself:

```bash
cmake -S bench -B build-bench -DCMAKE_BUILD_TYPE=Release && cmake --build build-bench
./build-bench/model_bench
./build-bench/perf_bench
python3 bench/report.py build-bench > benchmark-report.md   # the full report
```

> **Numbers vary.** Everything below is wall-clock time on a developer laptop,
> given as *orders of magnitude* — a faster or busier machine will differ. Use
> them for relative comparisons ("the MPC is ~1000× a plain model tick"), not as
> fixed figures. The CI artifact has the numbers for that specific run.

## 1. How long a tick takes, per model

One tick = one call to the model's `PerformIntegration`: run the control law and
advance the physics by 10 ms (for the MPC model, also re-solve the optimisation).

| model | typical tick | notes |
|-------|-------------:|-------|
| Rocket (feed-forward + LQR) | **~0.4 µs** | closed-form control + one RK4 step |
| Quadrotor (feed-forward + LQR) | **~0.9 µs** | same, with quaternion attitude |
| Quadrotor **MPC** | **~0.5 µs or ~5 ms** | *bimodal* — see below |

The MPC is different: it only re-solves its optimisation at the control cadence
(every 20 ms), and holds the last command in between. So most ticks are ~0.5 µs
(just applying the held command), and one tick in two is a full **~5 ms solve**
(p95 ≈ 6.5 ms). That is by design — the expensive solve is kept off most ticks so
a high tick rate cannot stall the simulation, and 5 ms comfortably fits the 20 ms
control period. It also means the MPC is the one model whose cost matters when
sizing the loop.

## 2. What the diagnostics cost

The logger, profiler and recorder are always compiled in but **off by default**
(log level Warn, no profiler module enabled, recording off). Left at their
defaults, each call site only does a cheap "is this on?" check:

| call | at default (off/filtered) | when turned on |
|------|--------------------------:|---------------:|
| a log line below the level | **~1 ns** (args not even evaluated) | ~45 ns to queue it |
| a profiler scope | **~5 ns** | ~210 ns (two clock reads) |
| a recorder row | **~0.6 ns** | ~68 ns to queue the row |

Two things worth knowing:

- **Logging is deferred.** Queuing a log line costs ~45 ns on the simulation
  thread; the expensive part — formatting the text (~285 ns) — happens later on a
  separate drain thread, so it never slows the tick.
- **Profiling isn't free when on.** A profiled scope costs ~210 ns (dominated by
  two clock reads), so profile coarse-grained scopes, not tight inner loops, and
  turn modules on only while you need them.

**Compiling them out.** For a clean release build the call sites can be stripped
entirely with `CDS_LOG_COMPILE_LEVEL`, `CDS_PROFILE_ENABLED=0`,
`CDS_RECORD_ENABLED=0` (see [build.md](build.md)); the residual then measures ~0.

## 3. wasm-only vs ws-served

The two deployments run the **same physics core**, so the per-model tick costs
above are identical in both. They differ only in how the browser talks to the
core:

- **wasm-only** — the core is compiled into the page; a frontend call is a direct
  in-process function call (embind), with no serialisation and no network hop, so
  the frontend↔core overhead is negligible.
- **ws-served** — the core runs as a native server and the browser is a thin
  client; every call is a WebSocket round-trip (marshalling + a worker hop +
  localhost/network latency), so it carries real transport overhead but lets the
  core run at native speed on a separate machine.

