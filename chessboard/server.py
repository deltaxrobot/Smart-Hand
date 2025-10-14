from __future__ import annotations

import argparse
import json
import mimetypes
import socket
from http import HTTPStatus
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
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
    # Thông tin tương thích cũ
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

        self._send_error(HTTPStatus.NOT_FOUND, "Không tìm thấy tài nguyên yêu cầu.")

    def log_message(self, format: str, *args) -> None:
        return  # Quiet server logging for cleaner console output.

    # Internal helpers -------------------------------------------------

    def _serve_index(self) -> None:
        if not INDEX_FILE.is_file():
            self._send_error(HTTPStatus.INTERNAL_SERVER_ERROR, "Thiếu tệp index.html.")
            return

        content = INDEX_FILE.read_text(encoding="utf-8").replace("{{LOCAL_IP}}", LOCAL_IP)
        self._send_bytes(content.encode("utf-8"), "text/html; charset=utf-8")

    def _serve_static(self, path: str) -> None:
        rel_path = path[len("/static/") :]
        file_path = (STATIC_DIR / rel_path).resolve()

        if not file_path.is_file() or STATIC_DIR not in file_path.parents:
            self._send_error(HTTPStatus.NOT_FOUND, "Không tìm thấy tệp tĩnh.")
            return

        mime_type, _ = mimetypes.guess_type(file_path.name)
        content_type = mime_type or "application/octet-stream"
        self._send_bytes(file_path.read_bytes(), f"{content_type}")

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
    parser = argparse.ArgumentParser(description="Server hiển thị bảng cờ hiệu chuẩn cho smartphone.")
    parser.add_argument(
        "--host",
        default="0.0.0.0",
        help="Địa chỉ lắng nghe (mặc định: 0.0.0.0).",
    )
    parser.add_argument(
        "--port",
        type=int,
        default=8080,
        help="Cổng lắng nghe (mặc định: 8080).",
    )
    return parser.parse_args()


def main() -> None:
    args = parse_args()
    server_address = (args.host, args.port)
    httpd = ThreadingHTTPServer(server_address, ChessboardRequestHandler)
    print(
        f"Server đang chạy tại http://{LOCAL_IP}:{args.port} (truy cập từ mạng nội bộ) "
        f"hoặc http://{args.host}:{args.port} (truy cập cục bộ)."
    )
    print("Nhấn Ctrl+C để dừng server.")
    try:
        httpd.serve_forever()
    except KeyboardInterrupt:
        print("\nĐang tắt server...")
    finally:
        httpd.server_close()


if __name__ == "__main__":
    main()
