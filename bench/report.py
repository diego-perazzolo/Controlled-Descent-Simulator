#!/usr/bin/env python3
# =============================================================================
# Controlled Descent Simulator
# =============================================================================
# File        : report.py
# Description : Assembles the benchmark artifact (a single Markdown document) by
#               running the bench binaries and wrapping their tables with plain
#               explanations aimed at non-experts. EPHEMERAL: regenerate it on
#               every run and upload it as a CI artifact — do NOT commit it, the
#               numbers vary by machine and would dirty the tree. The stable,
#               hand-curated companion is docs/benchmark.md.
# Usage       : python3 bench/report.py [BUILD_DIR] > benchmark-report.md
# =============================================================================
import datetime
import os
import platform
import subprocess
import sys

BUILD = sys.argv[1] if len(sys.argv) > 1 else "build-bench"


def run(binary):
    path = os.path.join(BUILD, binary)
    try:
        return subprocess.run([path], capture_output=True, text=True, timeout=300).stdout.strip()
    except Exception as e:  # never let a missing/hung binary sink the report
        return f"(could not run {path}: {e})"


def _cmd(args):
    try:
        return subprocess.run(args, capture_output=True, text=True, timeout=15).stdout
    except Exception:
        return ""


def _read(path):
    try:
        with open(path) as f:
            return f.read()
    except Exception:
        return ""


# Non-sensitive hardware summary for context (CPU model, core counts, RAM size
# and speed, CPU frequency). It NEVER emits serials/UUIDs: only these specific,
# non-identifying fields are extracted — the raw tool output is never printed.
def machine_info():
    sysname = platform.system()
    cpu = cores = threads = ram = cpu_ghz = ram_speed = ""

    if sysname == "Darwin":
        cpu = _cmd(["sysctl", "-n", "machdep.cpu.brand_string"]).strip()
        cores = _cmd(["sysctl", "-n", "hw.physicalcpu"]).strip()
        threads = _cmd(["sysctl", "-n", "hw.logicalcpu"]).strip()
        mem = _cmd(["sysctl", "-n", "hw.memsize"]).strip()
        if mem.isdigit():
            ram = f"{int(mem) / (1024 ** 3):.0f} GB"
        hz = _cmd(["sysctl", "-n", "hw.cpufrequency"]).strip()  # empty on Apple Silicon
        if hz.isdigit():
            cpu_ghz = f"{int(hz) / 1e9:.2f} GHz"
        # RAM speed: extract ONLY the "Speed:" line — never the serial-bearing dump
        for line in _cmd(["system_profiler", "SPMemoryDataType"]).splitlines():
            if "Speed:" in line:
                ram_speed = line.split("Speed:", 1)[1].strip()
                break

    elif sysname == "Linux":
        for line in _read("/proc/cpuinfo").splitlines():
            if line.lower().startswith("model name"):
                cpu = line.split(":", 1)[1].strip()
                break
        threads = _cmd(["nproc"]).strip()
        for line in _read("/proc/meminfo").splitlines():
            if line.startswith("MemTotal"):
                try:
                    ram = f"{int(line.split()[1]) / (1024 ** 2):.0f} GB"
                except Exception:
                    pass
                break
        for line in _read("/proc/cpuinfo").splitlines():
            if line.lower().startswith("cpu mhz"):
                try:
                    cpu_ghz = f"{float(line.split(':', 1)[1]) / 1000:.2f} GHz"
                except Exception:
                    pass
                break

    if not cpu:
        cpu = platform.processor() or "unknown CPU"

    bits = [cpu]
    if cores and threads:
        bits.append(f"{cores} cores / {threads} threads")
    elif threads:
        bits.append(f"{threads} threads")
    if cpu_ghz:
        bits.append(cpu_ghz)
    if ram:
        bits.append(f"{ram} RAM" + (f" @ {ram_speed}" if ram_speed else ""))
    return " · ".join(bits)


model = run("model_bench")
perf_in = run("perf_bench")
perf_off = run("perf_bench_off")

when = datetime.datetime.now().strftime("%Y-%m-%d %H:%M")
host = f"{platform.system()} {platform.machine()}"

print(f"""# CDS benchmark report

_Generated {when} on {host}._
_Machine: {machine_info()}._

These are wall-clock timings measured on the machine that produced this file.
**Absolute numbers vary** between machines and even between runs — read them as
orders of magnitude and relative comparisons, not fixed figures. The discussion
and the wasm-only vs served notes live in `docs/benchmark.md`.

## 1. Integration time per model

How long a single physics tick takes for each model — `PerformIntegration`:
control law + numerical integration, plus the Model-Predictive-Control solve for
the MPC model. This is the cost the simulation pays on every tick.

Reading it: `mean` is the average tick; `p95` is the slow 5% (for the MPC these
are the ticks that actually re-solve, so the value is much larger than the mean).

```
{model}
```

## 2. Impact of the diagnostics (logger / profiler / recorder)

The per-call cost of the diagnostics, so you can see what leaving them on costs.
At their defaults (log level Warn, profiler modules off, recording off) each call
site only does the cheap "is this on?" check — the *disabled / filtered* rows,
a few nanoseconds. Turning a feature on pays the *enabled* row; the drain (log
formatting) happens off the simulation thread.

Features compiled in (what ships):

```
{perf_in}
```

Same code with the features compiled out (`CDS_*` macros stripped) — the residual
is essentially zero, i.e. they can be removed entirely for a release build:

```
{perf_off}
```
""")
