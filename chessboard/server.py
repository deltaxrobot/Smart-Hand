# from __future__ import annotations  # Python 3.11+ has annotations by default

import argparse
import json
import mimetypes
import socket
from http import HTTPStatus
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
from typing import Tuple
from urllib.parse import unquote

BASE_DIR = Path(__file__).resolve().parent
STATIC_DIR = (BASE_DIR / "static").resolve()
INDEX_FILE = BASE_DIR / "index.html"


def get_local_ip() -> str:
    """Try to detect the primary local IP address."""
    try:
        with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as sock:
            sock.connect(("8.8.8.8", 80))
            return sock.getsockname()[0]
    except OSError:
        return "127.0.0.1"


LOCAL_IP = get_local_ip()

DEFAULT_CONFIG = {
    "size": 8,
    "squareSize": 60,
    "darkColor": "#000000",
    "lightColor": "#ffffff",
    "boardMargin": 20,
    "showCoordinates": False,
    "cols": 8,
    "rows": 8,
}


class ChessboardRequestHandler(BaseHTTPRequestHandler):
    server_version = "ChessboardServer/1.0"

    def do_GET(self) -> None:  # noqa: N802
        path = unquote(self.path.split("?", 1)[0])

        if path in {"/", "/index.html"}:
            self._serve_index()
            return

        if path.startswith("/static/"):
            self._serve_static(path)
            return

        if path == "/api/info":
            self._serve_info()
            return

        self._send_error(HTTPStatus.NOT_FOUND, "Resource not found.")

    def log_message(self, format: str, *args) -> None:
        return  # Silence default logging.

    # Internal helpers -------------------------------------------------

    def _serve_index(self) -> None:
        if not INDEX_FILE.is_file():
            self._send_error(HTTPStatus.INTERNAL_SERVER_ERROR, "Missing index.html.")
            return

        content = INDEX_FILE.read_text(encoding="utf-8").replace("{{LOCAL_IP}}", LOCAL_IP)
        self._send_bytes(content.encode("utf-8"), "text/html; charset=utf-8")

    def _serve_static(self, path: str) -> None:
        rel_path = path[len("/static/") :]
        file_path = (STATIC_DIR / rel_path).resolve()

        if not file_path.is_file() or STATIC_DIR not in file_path.parents:
            self._send_error(HTTPStatus.NOT_FOUND, "Static file not found.")
            return

        mime_type, _ = mimetypes.guess_type(file_path.name)
        content_type = mime_type or "application/octet-stream"
        self._send_bytes(file_path.read_bytes(), content_type)

    def _serve_info(self) -> None:
        payload = {
            "local_ip": LOCAL_IP,
            "default_config": DEFAULT_CONFIG,
        }
        data = json.dumps(payload).encode("utf-8")
        self._send_bytes(data, "application/json; charset=utf-8")

    def _send_bytes(self, data: bytes, content_type: str) -> None:
        self.send_response(HTTPStatus.OK)
        self.send_header("Content-Type", content_type)
        self.send_header("Content-Length", str(len(data)))
        self.send_header("Cache-Control", "no-cache, no-store, must-revalidate")
        self.send_header("X-Content-Type-Options", "nosniff")
        self.end_headers()
        self.wfile.write(data)

    def _send_error(self, status: HTTPStatus, message: str) -> None:
        body = json.dumps({"error": message}).encode("utf-8")
        self.send_response(status)
        self.send_header("Content-Type", "application/json; charset=utf-8")
        self.send_header("Content-Length", str(len(body)))
        self.end_headers()
        self.wfile.write(body)


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Serve the calibration chessboard for smartphone capture.")
    parser.add_argument(
        "--host",
        default="0.0.0.0",
        help="Host/IP to bind (default: 0.0.0.0).",
    )
    parser.add_argument(
        "--port",
        type=int,
        default=8080,
        help="Preferred port to bind (default: 8080).",
    )
    parser.add_argument(
        "--port-attempts",
        type=int,
        default=20,
        help="How many sequential ports to try if the preferred one is busy.",
    )
    parser.add_argument(
        "--status-file",
        help="Write a JSON status file containing the bound port and URLs.",
    )
    return parser.parse_args()


def create_server(host: str, port: int, attempts: int) -> Tuple[ThreadingHTTPServer, int]:
    """
    Try to start the HTTP server, probing sequential ports when the requested one is busy.
    Returns the running server instance and the port it actually bound to.
    """
    last_error: OSError | None = None
    for offset in range(max(1, attempts)):
        candidate_port = port + offset
        try:
            httpd = ThreadingHTTPServer((host, candidate_port), ChessboardRequestHandler)
            return httpd, candidate_port
        except OSError as exc:
            last_error = exc
            continue
    raise last_error if last_error else OSError("Unable to bind server to any port.")


def main() -> None:
    args = parse_args()
    try:
        httpd, bound_port = create_server(args.host, args.port, args.port_attempts)
    except OSError as exc:
        print(f"Failed to start server: {exc}")
        raise SystemExit(1) from exc

    status_payload = {
        "host": args.host,
        "port": bound_port,
        "local_ip": LOCAL_IP,
        "local_url": f"http://{LOCAL_IP}:{bound_port}",
        "bound_url": f"http://{args.host}:{bound_port}",
    }
    if args.status_file:
        status_path = Path(args.status_file).resolve()
        try:
            status_path.parent.mkdir(parents=True, exist_ok=True)
            status_path.write_text(json.dumps(status_payload), encoding="utf-8")
        except OSError as exc:
            print(f"Warning: unable to write status file '{status_path}': {exc}", flush=True)

    if bound_port != args.port:
        print(f"Port {args.port} unavailable; switched to port {bound_port}.", flush=True)

    print(
        f"Server running at http://{LOCAL_IP}:{bound_port} (local network) "
        f"or http://{args.host}:{bound_port} (bound interface).",
        flush=True,
    )
    print("Press Ctrl+C to stop the server.")
    try:
        httpd.serve_forever()
    except KeyboardInterrupt:
        print("\nStopping server...")
    finally:
        httpd.server_close()


if __name__ == "__main__":
    main()
