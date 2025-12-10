#!/usr/bin/env python3
"""
Button -> TCP Command Bridge

Usage:
  - Instantiate ButtonCommandBridge(host, port, mapping=...) and attach it to
    `DualFairinoCartRobot` by setting `dual_robot.button_bridge = bridge` before
    calling `dual_robot.run_vr_loop(...)`.

Behavior:
  - The bridge receives periodic calls to `process_status(status)` where
    `status` is the dict returned by `DualFairinoCartRobot.get_button_status`.
  - It detects press/release edges and sends mapped TCP commands once per
    edge. Mapping is configurable.
"""

import time
import logging
import threading
from typing import Dict, Optional, Any

logger = logging.getLogger(__name__)

DEFAULT_MAPPING = {
    # example mappings: on_press and on_release keys
    "A": {"on_press": "startDrill",     "on_release": "stopDrill"},
    "X": {"on_press": "slowShunScrew",  "on_release": "STOPSCREW"},
    "Y": {"on_press": "mediumShunScrew","on_release": "STOPSCREW"},
    "B": {"on_press": "mediumNiScrew",  "on_release": "STOPSCREW"},
}


class ButtonCommandBridge:
    def __init__(self, allowed_ip: str, port: int, mapping: Optional[Dict[str, Dict[str, str]]] = None, newline: bool = True, timeout: float = 1.0, tcp_server=None):
        """Bridge that sends button commands via a TcpServer from tools/TCP.py.

        Parameters:
        - allowed_ip: the client IP that will be allowed to connect (passed to TcpServer)
        - port: server port to bind
        - tcp_server: optional pre-created TcpServer instance
        """
        self.host = allowed_ip
        self.port = port
        self.newline = newline
        self.timeout = timeout
        self.mapping = mapping or DEFAULT_MAPPING
        self._last_pressed: Dict[str, bool] = {}

        # TcpServer integration: always use tcp_server (create if not provided)
        if tcp_server is not None:
            self._tcp_server = tcp_server
        else:
            # import lazily to avoid import-time dependency issues
            try:
                from tools.TCP import TcpServer
            except Exception:
                from .TCP import TcpServer

            self._tcp_server = TcpServer(self.port, self.host)
            # bind/listen synchronously, then accept in background
            self._tcp_server.start()
            self._accept_thread = threading.Thread(target=self._accept_background, daemon=True)
            self._accept_thread.start()

    

    def _send_raw(self, data: bytes) -> bool:
        raise NotImplementedError("_send_raw is removed; ButtonCommandBridge uses TcpServer.send_message")

    def send_command(self, cmd: str) -> bool:
        """Send command to MCU via TcpServer. Returns True on success."""
        out = cmd + ("\n" if self.newline else "")
        try:
            return bool(self._tcp_server.send_message(out))
        except Exception as e:
            logger.warning(f"TcpServer send failed: {e}")
            return False

    def _accept_background(self):
        try:
            logger.info("TcpServer listening on port %s, waiting for allowed client %s", self.port, self.host)
            self._tcp_server.accept_client()
            logger.info("TcpServer accepted client: %s", self._tcp_server.client_socket.getpeername())
        except Exception as e:
            logger.warning(f"TcpServer accept loop exited: {e}")

    def close(self):
        try:
            if hasattr(self, '_tcp_server') and self._tcp_server:
                self._tcp_server.close_socket()
        except Exception:
            pass

    def process_status(self, status: Optional[Dict[str, Dict[str, Any]]]):
        """
        Called periodically with the output of `get_button_status`.
        Detects edges and sends configured commands.
        """
        if not status:
            return
        for btn, cfg in self.mapping.items():
            s = status.get(btn)
            pressed = False
            if s is not None:
                pressed = bool(s.get("pressed", False))

            last = self._last_pressed.get(btn, False)
            # rising edge -> on_press
            if pressed and not last:
                cmd = cfg.get("on_press")
                if cmd:
                    sent = self.send_command(cmd)
                    logger.info(f"Button {btn} pressed -> {cmd} (sent={sent})")
            # falling edge -> on_release
            if (not pressed) and last:
                cmd = cfg.get("on_release")
                if cmd:
                    sent = self.send_command(cmd)
                    logger.info(f"Button {btn} released -> {cmd} (sent={sent})")

            self._last_pressed[btn] = pressed


if __name__ == "__main__":
    import argparse

    p = argparse.ArgumentParser(description="Button -> TCP Command Bridge (test)")
    # Default to MCU IP so the bridge connects to the MCU server by default
    p.add_argument("--allowed-ip", dest="allowed_ip", default="192.168.0.111",
                   help="allowed client IP to accept (default: 192.168.0.111)")
    p.add_argument("--port", type=int, default=5001, help="TCP port to bind and accept client on (default: 5001)")
    # The ButtonCommandBridge constructor appends a newline by default. Expose
    # a --no-newline flag to disable that behavior from the CLI.
    p.add_argument("--no-newline", action="store_false", dest="newline",
                   help="do not append newline to commands (default: append newline)")
    args = p.parse_args()

    logging.basicConfig(level=logging.DEBUG)

    # Always run as server using tools/TCP.TcpServer
    try:
        from tools.TCP import TcpServer
    except Exception:
        from TCP import TcpServer

    tcp_server = TcpServer(args.port, args.allowed_ip)
    if not tcp_server.start():
        logger.error("Failed to start TcpServer")
        raise SystemExit(2)
    logger.info("Waiting for MCU (client) to connect...")
    try:
        tcp_server.accept_client()
    except KeyboardInterrupt:
        logger.info("Interrupted while waiting for client")
        tcp_server.close_socket()
        raise

    bridge = ButtonCommandBridge(args.allowed_ip, args.port, newline=args.newline, tcp_server=tcp_server)
    # quick manual demo loop that prints and sends on press edges
    try:
        logger.info(f"Starting demo bridge -> {args.allowed_ip}:{args.port} newline={args.newline}")
        while True:
            # demo: no real status; keep loop alive so user can manually call bridge.process_status
            time.sleep(0.1)
    except KeyboardInterrupt:
        bridge.close()
