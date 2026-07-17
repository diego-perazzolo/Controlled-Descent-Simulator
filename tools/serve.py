# =============================================================================
# Controlled Descent Simulator
# =============================================================================
# File        : serve.py
# Description : Local dev server for the frontend. Same as
#               `python3 -m http.server` but adds the COOP/COEP headers that
#               make the page cross-origin isolated, which is required for
#               SharedArrayBuffer (used by the WebSocket backend proxy).
#               Works for the local-core build too.
# Usage       : python3 tools/serve.py [port]   (default 8080, serves repo root)
# =============================================================================
import http.server
import os
import sys


class CoopCoepHandler(http.server.SimpleHTTPRequestHandler):
    def end_headers(self):
        self.send_header("Cross-Origin-Opener-Policy", "same-origin")
        self.send_header("Cross-Origin-Embedder-Policy", "require-corp")
        self.send_header("Cache-Control", "no-store")
        super().end_headers()


def main():
    port = int(sys.argv[1]) if len(sys.argv) > 1 else 8080
    root = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
    os.chdir(root)

    server = http.server.ThreadingHTTPServer(("0.0.0.0", port), CoopCoepHandler)
    print(f"Serving {root} at http://localhost:{port}/frontend/ (COOP/COEP enabled)")
    try:
        server.serve_forever()
    except KeyboardInterrupt:
        print("\nStopped.")
    finally:
        server.server_close()


if __name__ == "__main__":
    main()
