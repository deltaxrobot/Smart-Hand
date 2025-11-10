"""Utility helpers for the SmartHand application."""

import socket

MAX_TRANSFORM_DIMENSION = 4096  # Clamp warped outputs to a sane size


def get_local_ip() -> str:
    """Best-effort detection of the host's primary IPv4 address."""
    try:
        with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as sock:
            sock.connect(("8.8.8.8", 80))
            return sock.getsockname()[0]
    except OSError:
        return "127.0.0.1"


__all__ = ["MAX_TRANSFORM_DIMENSION", "get_local_ip"]
