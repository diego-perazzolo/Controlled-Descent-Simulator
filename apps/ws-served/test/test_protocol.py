# =============================================================================
# Controlled Descent Simulator
# =============================================================================
# File        : test_protocol.py
# Description : End-to-end test of the ws-served wire protocol against a real
#               cds_server: WebSocket handshake, transport correlation id,
#               protocol version check (positive and negative), and the full
#               command round-trip (init rocket, append poly4, get point,
#               set system params, run / snapshot / stop, remove last).
#               Trajectory values are checked against the reference descent of
#               the design notebook; the simulation itself advances on the
#               server's real-time thread, so time checks use wall-clock
#               bounds instead of exact positions.
#               The expected protocol version is parsed from the generated
#               ws_protocol.hpp, so the test follows the API description.
# Usage       : python3 apps/ws-served/test/test_protocol.py [path/to/cds_server]
#               With no argument, expects a server already listening on 9102.
# =============================================================================
import base64
import hashlib
import os
import re
import socket
import struct
import subprocess
import sys
import time

HOST, PORT = "127.0.0.1", 9102
REPO = os.path.dirname(os.path.dirname(os.path.dirname(
    os.path.dirname(os.path.abspath(__file__)))))


def protocol_version():
    src = open(os.path.join(
        REPO, "apps/ws-served/exported_cpp/ws_protocol.hpp")).read()
    return int(re.search(r"WS_PROTOCOL_VERSION = 0x([0-9A-Fa-f]+)", src).group(1), 16)


def msg_ids():
    """Message ids parsed from the generated header, so the test follows the
    API description instead of hardcoding numbers."""
    src = open(os.path.join(
        REPO, "apps/ws-served/exported_cpp/ws_protocol.hpp")).read()
    return {m.group(1): int(m.group(2))
            for m in re.finditer(r"WS_MSG_(\w+)\s*=\s*(\d+)", src)}


VERSION = protocol_version()
MSG = msg_ids()


# --------------------------------------------------------------------------- #
# minimal WebSocket client (RFC 6455, client side)                             #
# --------------------------------------------------------------------------- #

def ws_connect(retry_s=10):
    deadline = time.time() + retry_s
    while True:
        try:
            s = socket.create_connection((HOST, PORT), timeout=5)
            break
        except OSError:
            if time.time() > deadline:
                raise
            time.sleep(0.2)
    key = base64.b64encode(os.urandom(16)).decode()
    req = (f"GET / HTTP/1.1\r\nHost: {HOST}:{PORT}\r\nUpgrade: websocket\r\n"
           f"Connection: Upgrade\r\nSec-WebSocket-Key: {key}\r\n"
           f"Sec-WebSocket-Version: 13\r\n\r\n")
    s.sendall(req.encode())
    resp = b""
    while b"\r\n\r\n" not in resp:
        resp += s.recv(1024)
    expect = base64.b64encode(hashlib.sha1(
        (key + "258EAFA5-E914-47DA-95CA-C5AB0DC85B11").encode()).digest()).decode()
    assert b"101" in resp.split(b"\r\n")[0], resp
    assert expect.encode() in resp, "bad Sec-WebSocket-Accept token"
    return s


def send_frame(s, payload):
    mask = os.urandom(4)
    n = len(payload)
    hdr = bytes([0x82])
    if n < 126:
        hdr += bytes([0x80 | n])
    else:
        hdr += bytes([0x80 | 126]) + struct.pack(">H", n)
    masked = bytes(b ^ mask[i % 4] for i, b in enumerate(payload))
    s.sendall(hdr + mask + masked)


def recv_exact(s, n):
    buf = b""
    while len(buf) < n:
        chunk = s.recv(n - len(buf))
        assert chunk, "connection closed by server"
        buf += chunk
    return buf


def recv_frame(s):
    h = recv_exact(s, 2)
    ln = h[1] & 0x7F
    if ln == 126:
        ln = struct.unpack(">H", recv_exact(s, 2))[0]
    return h[0] & 0x0F, recv_exact(s, ln)


# --------------------------------------------------------------------------- #
# transport framing: [u32 correlation id LE][payload], id echoed back          #
# --------------------------------------------------------------------------- #

_next_id = 0


def rpc(s, payload):
    global _next_id
    _next_id += 1
    send_frame(s, struct.pack("<I", _next_id) + payload)
    _, p = recv_frame(s)
    rid = struct.unpack("<I", p[:4])[0]
    assert rid == _next_id, f"correlation mismatch: {rid} != {_next_id}"
    return p[4:]


def header(msg_type, version=None):
    return struct.pack("<BB", VERSION if version is None else version, msg_type)


# --------------------------------------------------------------------------- #
# test cases                                                                   #
# --------------------------------------------------------------------------- #

def run_tests(s):
    # version mismatch must be rejected with isError=1 and the server version
    p = rpc(s, header(MSG["INIT_ROCKET"], version=(VERSION + 1) & 0xFF) + b"\x00" * 56)
    v, t, err = struct.unpack("<BBB", p)
    assert (v, err) == (VERSION, 1), f"version mismatch not rejected: v={v} err={err}"
    print(f"version mismatch rejected OK (server 0x{v:02X})")

    # init rocket: 6 floats params + 8 floats actuator limits
    params = struct.pack("<6f", 10.0, 10 / 3, 10 / 3, 1.0, 1.0, 0.02)
    limits = struct.pack("<8f", 500, 0, 10, -10, 10, -10, 10, -10)
    p = rpc(s, header(MSG["INIT_ROCKET"]) + params + limits)
    v, t, err = struct.unpack("<BBB", p)
    assert (v, t, err) == (VERSION, 1, 0), f"init rocket failed: {v} {t} {err}"
    print("init rocket OK")

    # append poly4: 21 floats (reference descent of the design notebook)
    poly = struct.pack("<21f",
                       -50, 50, 80, 0,
                       0, 5, -50, 0,
                       0, 0, 0, 0,
                       0, 0, 0, 0,
                       0, 0, 0, 0,
                       20)
    p = rpc(s, header(MSG["TRAJ_APPEND_POLY4"]) + poly)
    assert struct.unpack("<BBB", p)[2] == 0, "append poly4 failed"
    print("append poly4 OK")

    # trajectory point at t=10 must match the reference values
    p = rpc(s, header(MSG["TRAJ_GET_POINT"]) + struct.pack("<f", 10.0))
    v, t, x, y, z = struct.unpack("<BB3f", p)
    assert (round(x, 2), round(y, 2), round(z, 2)) == (-15.62, 21.88, -37.50), \
        f"trajectory point drifted: ({x:.2f}, {y:.2f}, {z:.2f})"
    print(f"trajectory point OK ({x:.2f}, {y:.2f}, {z:.2f})")

    # system params: 10 ms tick period, no user forces
    p = rpc(s, header(MSG["SET_SYSTEM_PARAMS"]) + struct.pack("<4f", 0.01, 0, 0, 0))
    assert struct.unpack("<BBB", p)[2] == 0, "set system params failed"
    print("set system params OK")

    # snapshot while stopped: no error, simulated time still at zero
    p = rpc(s, header(MSG["GET_SNAPSHOT"]))
    vals = struct.unpack("<BB17fB", p)
    assert vals[-1] == 0, "snapshot returned error"
    assert vals[2] == 0.0, f"time advanced before run: {vals[2]}"
    print("snapshot at rest OK (t=0)")

    # plant snapshot before run: the server attaches a loopback plant at boot
    # and its link is up from attach (two-phase lifecycle), so it already
    # streams a hover — a sample may or may not have arrived yet by now, we
    # only assert the plant is attached (a "no sample" check would race the
    # first hover publish)
    p = rpc(s, header(MSG["GET_PLANT_SNAPSHOT"]))
    vals = struct.unpack("<BB14fBBB", p)  # ...isAttached, isReadyToStart, isError
    assert vals[-3] == 1, "plant not attached"
    print("plant snapshot before run OK (attached)")

    # run for ~0.5 s of wall time: simulated time must advance with the
    # real-time thread (loose bounds, the tick pace is not deterministic)
    p = rpc(s, header(MSG["RUN"]))
    assert struct.unpack("<BBB", p)[2] == 0, "run failed"
    time.sleep(0.5)
    p = rpc(s, header(MSG["GET_SNAPSHOT"]))
    vals = struct.unpack("<BB17fB", p)
    assert vals[-1] == 0, "snapshot returned error"
    t_run = vals[2]
    assert 0.05 < t_run < 2.0, f"simulated time not advancing: {t_run}"
    print(f"run + snapshot OK (t={t_run:.3f}s)")

    # plant snapshot while running: fresh samples with growing sequence
    # (loopback: period 20 ms, latency 50 ms — first sample after ~70 ms)
    p = rpc(s, header(MSG["GET_PLANT_SNAPSHOT"]))
    pv = struct.unpack("<BB14fBBB", p)
    assert (pv[-3], pv[-1]) == (1, 0), f"plant snapshot failed: {pv[-3:]}"
    seq_run, t_plant = pv[3], pv[2]
    assert seq_run >= 1, f"plant sequence not started: {seq_run}"
    assert t_plant > 0, f"plant time not advancing: {t_plant}"
    print(f"plant snapshot OK (seq={seq_run:.0f}, plantTime={t_plant:.3f}s)")

    # stop: simulated time must freeze
    p = rpc(s, header(MSG["STOP"]))
    assert struct.unpack("<BBB", p)[2] == 0, "stop failed"
    t1 = struct.unpack("<BB17fB", rpc(s, header(MSG["GET_SNAPSHOT"])))[2]
    time.sleep(0.2)
    t2 = struct.unpack("<BB17fB", rpc(s, header(MSG["GET_SNAPSHOT"])))[2]
    assert t1 == t2, f"time still advancing after stop: {t1} -> {t2}"
    print(f"stop OK (t frozen at {t2:.3f}s)")

    # plant after stop: STOP ends the mission, not the link. The plant stays
    # connected (two-phase lifecycle: link lives from attach to detach), so a
    # loopback keeps publishing its held state and the sequence keeps growing.
    s1 = struct.unpack("<BB14fBBB", rpc(s, header(MSG["GET_PLANT_SNAPSHOT"])))[3]
    time.sleep(0.2)
    s2 = struct.unpack("<BB14fBBB", rpc(s, header(MSG["GET_PLANT_SNAPSHOT"])))[3]
    assert s2 > s1, f"plant link died on mission stop: seq {s1} -> {s2}"
    print(f"plant link alive after stop OK (seq {s1:.0f} -> {s2:.0f})")

    # remove last trajectory item
    p = rpc(s, header(MSG["TRAJ_REMOVE_LAST"]))
    assert struct.unpack("<BBB", p)[2] == 0, "remove last failed"
    print("remove last OK")

    # --- diagnostics: logger / profiler inspection (plumbing round-trip) ---
    # No core call-sites are instrumented yet, so these return well-formed but
    # empty text-blob payloads. We assert the exact wire sizes (char buffers on
    # the POD wire) and that the fields decode, plus the setters' out-of-range
    # error path.
    p = rpc(s, header(MSG["GET_LOG_MODULES"]))          # header(2) + char[1200] + count(f)
    assert len(p) == 1206, f"log modules size {len(p)}"
    assert (p[0], p[1]) == (VERSION, MSG["GET_LOG_MODULES"]), "log modules header"
    n_logmod = struct.unpack("<f", p[1202:1206])[0]
    print(f"get log modules OK (count={n_logmod:.0f})")

    p = rpc(s, header(MSG["GET_PROFILE_MODULES"]))       # same shape as log modules
    assert len(p) == 1206, f"profile modules size {len(p)}"
    print("get profile modules OK")

    p = rpc(s, header(MSG["GET_LOG_BATCH"]))            # header(2) + char[3800] + count + dropped
    assert len(p) == 3810, f"log batch size {len(p)}"
    n_lines = struct.unpack("<f", p[3802:3806])[0]
    dropped = struct.unpack("<f", p[3806:3810])[0]
    print(f"get log batch OK (count={n_lines:.0f}, dropped={dropped:.0f})")

    p = rpc(s, header(MSG["GET_PROFILE_TABLE"]))        # header(2) + char[3600] + count
    assert len(p) == 3606, f"profile table size {len(p)}"
    print("get profile table OK")

    # setters on a clearly out-of-range module must return error (the core
    # registers a couple of modules at startup, so index 999 is safely invalid).
    # setLogLevel carries {module, level, sampleN}
    p = rpc(s, header(MSG["SET_LOG_LEVEL"]) + struct.pack("<fff", 999.0, 0.0, 1.0))
    assert struct.unpack("<BBB", p)[2] == 1, "set log level should reject out-of-range module"
    print("set log level out-of-range rejected OK")

    p = rpc(s, header(MSG["SET_PROFILE_ENABLED"]) + struct.pack("<fB", 999.0, 1))
    assert struct.unpack("<BBB", p)[2] == 1, "set profile enabled should reject out-of-range module"
    print("set profile enabled out-of-range rejected OK")

    # reset profiler stats: always succeeds
    p = rpc(s, header(MSG["RESET_PROFILE"]))
    assert struct.unpack("<BBB", p)[2] == 0, "reset profile failed"
    print("reset profile OK")

    # toggle diag files off (two uint8 bools): server-side, always succeeds
    p = rpc(s, header(MSG["SET_DIAG_FILES"]) + struct.pack("<BB", 0, 0))
    assert struct.unpack("<BBB", p)[2] == 0, "set diag files failed"
    print("set diag files OK")

    # data recorder status: char[64] modelName + 3 floats (active, enabled,
    # droppedRows). No model is running under this test, so active must be 0.
    def record_status(p):
        assert len(p) == 2 + 64 + 12, f"record status size {len(p)}"
        name = p[2:66].split(b"\x00", 1)[0].decode("ascii", "replace")
        active, enabled, dropped = struct.unpack("<fff", p[66:78])
        return name, active, enabled, dropped

    # a model was initialized earlier in this test, so its recorder is active
    p = rpc(s, header(MSG["GET_RECORD_STATUS"]))
    name, active, enabled, dropped = record_status(p)
    assert active == 1.0, "the initialized model should have an active recorder"
    print(f"get record status OK (model={name!r}, active={active:.0f}, dropped={dropped:.0f})")

    # enable then disable recording; with a model active this really toggles
    p = rpc(s, header(MSG["SET_RECORDING"]) + struct.pack("<B", 1))
    _, active, enabled, _ = record_status(p)
    assert active == 1.0 and enabled == 1.0, "recording should enable with a model active"
    p = rpc(s, header(MSG["SET_RECORDING"]) + struct.pack("<B", 0))
    _, _, enabled, _ = record_status(p)
    assert enabled == 0.0, "recording should disable again"
    print("set recording toggle OK")

    # controller parameters: the Rocket initialized earlier exposes its LQR cost
    # diagonal (16 Q + 4 R = 20 rows). Read the manifest, set a weight by id, and
    # confirm the change round-trips; a bad id must be rejected.
    def manifest():
        p = rpc(s, header(MSG["GET_CONTROLLER_MANIFEST"]))
        assert len(p) == 2050, f"manifest size {len(p)}"
        text = p[2:2050].split(b"\x00", 1)[0].decode("ascii", "replace")
        rows = [ln.split("\t") for ln in text.splitlines() if ln]
        return rows

    rows = manifest()
    assert len(rows) == 20, f"expected 20 controller params, got {len(rows)}"
    assert rows[0][1] == "Q" and rows[0][3] == "rw", f"unexpected first row {rows[0]}"
    old = float(rows[0][4])
    p = rpc(s, header(MSG["SET_CONTROLLER_PARAM"]) + struct.pack("<2f", 0.0, 5000.0))
    assert struct.unpack("<BBB", p)[2] == 0, "set controller param (id 0) failed"
    assert abs(float(manifest()[0][4]) - 5000.0) < 1e-3, "controller param did not update"
    p = rpc(s, header(MSG["SET_CONTROLLER_PARAM"]) + struct.pack("<2f", 999.0, 1.0))
    assert struct.unpack("<BBB", p)[2] == 1, "out-of-range controller id should be rejected"
    rpc(s, header(MSG["SET_CONTROLLER_PARAM"]) + struct.pack("<2f", 0.0, old))  # restore
    print(f"controller params OK (20 rows, set/round-trip/reject)")

    # close politely
    s.sendall(bytes([0x88, 0x80]) + os.urandom(4))


def main():
    server = None
    if len(sys.argv) > 1:
        server = subprocess.Popen([os.path.abspath(sys.argv[1]), str(PORT)])
    try:
        s = ws_connect()
        print(f"handshake OK (protocol version 0x{VERSION:02X})")
        run_tests(s)
        print("ALL OK")
    finally:
        if server:
            server.terminate()
            server.wait()


if __name__ == "__main__":
    main()
