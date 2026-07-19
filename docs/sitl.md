# Running against ArduPilot SITL

The `sitl` plant drives an ArduPilot **Copter** SITL over MAVLink 2 / UDP and
mirrors it back into the simulator as the *plant ghost*, next to the ideal
model. This is the end-to-end walkthrough for wiring the plant to a SITL
running in **Docker** and flying a mission through the staging workflow.

For the plant's place in the architecture and the build commands, see
[`build.md`](build.md); this page picks up from a working build.

## Assumptions

- Docker Desktop is running and you already have a **working ArduCopter SITL
  container** (building the image / container is out of scope here).
- The **ws-served** app is built — both the WASM proxy and `cds_server` — as
  described in [`build.md`](build.md#ws-served-native-core-server--thin-client).
- The **QuadRotor** model is selected in the frontend (the plant is a copter).
- **Docker:** the container reaches the host through the
  special name `host.docker.internal`.

## Topology

The plant is a ground-control-station peer that **binds** `0.0.0.0:14550` and
learns the vehicle from the first valid datagram. QGroundControl is optional
and only for monitoring; give it its **own** UDP output so it does not share a
port or a system id with the plant.

```
 ┌──────────────────── Docker container ────────────────────┐
 │   ArduCopter SITL   (sim_vehicle.py + MAVProxy console)   │
 └───────────────┬───────────────────────────┬───────────────┘
        UDP out  │                            │  UDP out
   host.docker.internal:14550     host.docker.internal:14551
                 │                            │
        ┌────────▼─────────┐        ┌─────────▼─────────┐
        │  cds_server sitl │        │  QGroundControl   │
        │  bind :14550     │        │   (optional)      │
        │  sysid 254       │        │   sysid 255       │
        └──────────────────┘        └───────────────────┘
```

The plant identifies on the wire as **sysid 254** (component *mission
planner*), deliberately **not** the 255 that ground stations (QGroundControl,
MAVProxy) use: sharing 255/190 with a co-connected GCS makes the flight
controller see one conflated node and arbitrate commands erratically. That is
why QGC gets a separate output (`:14551`) rather than sharing the plant's link.

## Steps

**1. Start SITL in the container** and fan its MAVLink stream out to both host
ports (one for the plant, one for QGC):

```bash
sim_vehicle.py -v ArduCopter -f quad -I0 \
    --out=udp:host.docker.internal:14550 \
    --out=udp:host.docker.internal:14551
```

`-I0` selects instance 0; the two `--out` give the plant and QGC independent
links.

**2. Raise the telemetry stream rate** in the MAVProxy console. The plant needs
`LOCAL_POSITION_NED` and `ATTITUDE` flowing to build the ghost:

```
set streamrate 50
```

The plant also requests those two messages explicitly via
`SET_MESSAGE_INTERVAL` (re-requested until they arrive), but bumping the global
rate keeps the ghost smooth.

**3. Start the server with the `sitl` plant** on the host:

```bash
./build-server/cds_server 9002 sitl
```

It binds `0.0.0.0:14550` and waits for telemetry; the link reaches **READY** on
the autopilot heartbeat.

**4. (Optional) attach QGroundControl** on UDP `14551` to watch — or manually
fly — the vehicle alongside the plant.

**5. Serve the frontend** with the COOP/COEP headers (required by the WASM
proxy — see [`build.md`](build.md#ws-served-native-core-server--thin-client))
and open it:

```bash
python3 tools/serve.py 8080
```

Open `http://localhost:8080/frontend/` and select the **QuadRotor** model.

## Flying a mission

Once telemetry flows, the plant ghost appears in the 3D view and the **Plant
bar** shows up at the top of the frontend. From there:

1. **Set the altitude margin** (field *Altitude margin*, default `5` m). The
   staging altitude is the trajectory's vertical range (`max − min`) **plus**
   this margin — additive so that a flat trajectory still gets a valid climb.
2. **Begin staging.** The plant drives the vehicle `GUIDED → arm → takeoff →
   climb`; the status moves *Not staged → Staging… → Staged — ready*. If the
   vehicle is already airborne, staging skips the takeoff and climbs in place
   to the target altitude via setpoints.
3. **Start** runs the mission. Start is **gated on *Staged — ready*** whenever a
   plant is connected. At the first command the plant frame is aligned to the
   trajectory's first point (before Start the vehicle's motion is shown zeroed
   at the CDS origin).
4. **Stop / disconnect holds** the vehicle in place. Stop also **drops
   staging**, so before the next descent press *Begin staging* again to climb
   back to the target altitude — this guarantees every descent starts from the
   right height rather than wherever the previous run ended.

## MAVLink version pin

> The MAVLink C headers under `plants/sitl/mavlink/` are vendored and pinned;
> see [`plants/sitl/mavlink/VENDORED.md`](../plants/sitl/mavlink/VENDORED.md).
> `plants/sitl/mavlink_pin.hpp` fails the build if a re-vendor drifts the wire
> contract of the messages the plant uses.
