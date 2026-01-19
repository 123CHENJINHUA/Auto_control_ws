import socket
import logging
import time
from typing import Dict, Any, Optional


logger = logging.getLogger(__name__)


class ButtonCommandBridge:
    """
    Bridge VR button press/release events to string commands sent to a
    remote tool/device over TCP.

    Usage:
      mapping = {
        'X': {'on_press': 'mediumNiScrew',  'on_release': 'STOPSCREW'},
        'Y': {'on_press': 'mediumShunScrew','on_release': 'STOPSCREW'},
      }
      bridge = ButtonCommandBridge('192.168.0.10', 5001, mapping)
      ...
      bridge.process_status(status_dict)
    
    The status dict format is expected to be like:
      { 'A': {'value': float, 'pressed': bool}, 'B': {...}, ... }
    """

    def __init__(
        self,
        host: str,
        port: int,
        mapping: Dict[str, Dict[str, str]],
        *,
        newline: bool = True,
        connect_timeout: float = 1.5,
        send_timeout: float = 1.5,
        cooldown: float = 0.05,
    ) -> None:
        self._host = host
        self._port = int(port)
        self._mapping = mapping or {}
        self._newline = bool(newline)
        self._connect_timeout = float(connect_timeout)
        self._send_timeout = float(send_timeout)
        self._cooldown = float(cooldown)
        # Track last pressed state to detect rising/falling edges
        self._last_pressed: Dict[str, bool] = {}
        # Last send time for simple cooldown
        self._last_send_time: float = 0.0

    def _should_cooldown(self) -> bool:
        if self._cooldown <= 0:
            return False
        return (time.time() - self._last_send_time) < self._cooldown

    def _send(self, message: str) -> None:
        if not isinstance(message, str):
            message = str(message)
        if self._newline and not message.endswith("\n"):
            message_to_send = message + "\n"
        else:
            message_to_send = message

        try:
            with socket.create_connection((self._host, self._port), timeout=self._connect_timeout) as s:
                s.settimeout(self._send_timeout)
                s.sendall(message_to_send.encode('utf-8'))
                self._last_send_time = time.time()
        except Exception as e:
            logger.warning("ButtonCommandBridge send failed: %s (msg=%r)", e, message)

    def send_command(self, command: Optional[str]) -> None:
        if not command:
            return
        if self._should_cooldown():
            # Avoid spamming if something loops unexpectedly
            return
        self._send(command)

    def process_status(self, status: Dict[str, Dict[str, Any]]) -> None:
        """
        Inspect button status dict and fire mapped commands on
        press/release edge events.
        """
        if not isinstance(status, dict):
            return
        for btn, rules in self._mapping.items():
            try:
                cur_pressed = bool(status.get(btn, {}).get('pressed', False))
            except Exception:
                cur_pressed = False

            prev_pressed = self._last_pressed.get(btn, False)
            # Rising edge: on_press
            if cur_pressed and not prev_pressed:
                cmd = rules.get('on_press') if isinstance(rules, dict) else None
                if cmd:
                    logger.info("Button %s pressed -> send %r", btn, cmd)
                    self.send_command(cmd)
            # Falling edge: on_release
            if (not cur_pressed) and prev_pressed:
                cmd = rules.get('on_release') if isinstance(rules, dict) else None
                if cmd:
                    logger.info("Button %s released -> send %r", btn, cmd)
                    self.send_command(cmd)

            self._last_pressed[btn] = cur_pressed
