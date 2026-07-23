# Vendored MAVLink C headers — DO NOT HAND-EDIT

Everything under this directory (except this file) is **third-party generated
code**, vendored verbatim. Treat it exactly like an `exported_cpp/` folder:
never edit it by hand — to change it, re-vendor at a new upstream commit as
described below.

## Provenance

- Upstream: <https://github.com/mavlink/c_library_v2> (pregenerated MAVLink 2
  C headers, produced by mavgen from the
  [mavlink/mavlink](https://github.com/mavlink/mavlink) message definitions)
- Pinned commit: `622cfebf2137c47461bdc659f1e530d7a0f4dc9c`
  (master, upstream build date "Thu Jul 16 2026"; vendored 2026-07-18)
- License: MIT (the generated headers; upstream message definitions are CC0),
  see the upstream repositories.

## What is vendored

- Root helper headers: `checksum.h`, `protocol.h`, `mavlink_types.h`,
  `mavlink_helpers.h`, `mavlink_conversions.h`, `mavlink_get_info.h`,
  `mavlink_sha256.h`
- Dialects: `common/` (entry point used by the SITL plant) plus its include
  chain `standard/` and `minimal/`

Excluded on purpose: every other dialect and the per-dialect `testsuite.h`
files (only used by MAVLink's own test builds).

## Compatibility pinning

`plants/sitl/mavlink_pin.hpp` is the **only** include entry point for these
headers and `static_assert`s the wire contract of every message and command
number the plant uses. If a re-vendor drifts any of them, the build fails —
that is intentional. Only after reviewing the upstream change may the pins be
updated together with this file.

## How to re-vendor

```bash
SHA=<new upstream commit>
curl -sL https://github.com/mavlink/c_library_v2/archive/$SHA.tar.gz | tar xz
cd c_library_v2-$SHA
cp *.h                       <repo>/plants/sitl/mavlink/
rsync -a --exclude testsuite.h common standard minimal \
                             <repo>/plants/sitl/mavlink/
```

Then update the pinned commit in this file and rebuild: if
`mavlink_pin.hpp` fires, review the upstream protocol change before touching
the pins.
