# =============================================================================
# Controlled Descent Simulator
# =============================================================================
# File        : test_protocol.py
# Description : End-to-end test of the ws-served wire protocol against a real
#               cds_server: WebSocket handshake, transport correlation id,
#               protocol version check (positive and negative), and the full
#               command round-trip (init rocket, append poly4, get point,
#               100 steps, remove last). Values are checked against the
#               reference descent of the design notebook.
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


VERSION = protocol_version()


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
    p = rpc(s, header(1, version=(VERSION + 1) & 0xFF) + b"\x00" * 56)
    v, t, err = struct.unpack("<BBB", p)
    assert (v, err) == (VERSION, 1), f"version mismatch not rejected: v={v} err={err}"
    print(f"version mismatch rejected OK (server 0x{v:02X})")

    # init rocket: 6 floats params + 8 floats actuator limits
    params = struct.pack("<6f", 10.0, 10 / 3, 10 / 3, 1.0, 1.0, 0.02)
    limits = struct.pack("<8f", 500, 0, 10, -10, 10, -10, 10, -10)
    p = rpc(s, header(1) + params + limits)
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
    p = rpc(s, header(5) + poly)
    assert struct.unpack("<BBB", p)[2] == 0, "append poly4 failed"
    print("append poly4 OK")

    # trajectory point at t=10 must match the reference values
    p = rpc(s, header(4) + struct.pack("<f", 10.0))
    v, t, x, y, z = struct.unpack("<BB3f", p)
    assert (round(x, 2), round(y, 2), round(z, 2)) == (-15.62, 21.88, -37.50), \
        f"trajectory point drifted: ({x:.2f}, {y:.2f}, {z:.2f})"
    print(f"trajectory point OK ({x:.2f}, {y:.2f}, {z:.2f})")

    # 100 integration steps, check the reference position
    for _ in range(100):
        p = rpc(s, header(3) + struct.pack("<4f", 0.01, 0, 0, 0))
    vals = struct.unpack("<BBB16f", p)
    err, state = vals[2], vals[3:15]
    assert err == 0, "step returned error"
    pos = (round(state[3], 2), round(state[4], 2), round(state[5], 2))
    assert pos == (-49.45, 53.79, 36.03), f"simulation drifted: {pos}"
    print(f"100 steps OK pos={pos}")

    # remove last trajectory item
    p = rpc(s, header(7))
    assert struct.unpack("<BBB", p)[2] == 0, "remove last failed"
    print("remove last OK")

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
