import json
import logging
from typing import Any, Dict, Tuple

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

from .config import create_default_bridge, get_default_config
from .bridge import ButtonCommandBridge


logger = logging.getLogger(__name__)


class ButtonCommandBridgeNode(Node):
    def __init__(self) -> None:
        super().__init__('button2end')

        defaults = get_default_config()

        self.declare_parameter('host', defaults['host'])
        self.declare_parameter('port', int(defaults['port']))
        self.declare_parameter('newline', bool(defaults.get('newline', True)))
        self.declare_parameter('connect_timeout', float(defaults.get('connect_timeout', 1.5)))
        self.declare_parameter('send_timeout', float(defaults.get('send_timeout', 1.5)))
        self.declare_parameter('cooldown', float(defaults.get('cooldown', 0.05)))
        # ROS 2 parameters don't support nested dicts natively; store mapping as JSON string
        self.declare_parameter('mapping_json', json.dumps(defaults.get('mapping', {})))
        # VR actions topic carries the full action dict as JSON (including 'buttons')
        self.declare_parameter('actions_topic', '/vr/actions')
        # Threshold to consider an analog value as pressed
        self.declare_parameter('press_threshold', 0.5)

        cfg: Dict[str, Any] = {
            'host': self.get_parameter('host').get_parameter_value().string_value or str(defaults['host']),
            'port': int(self.get_parameter('port').get_parameter_value().integer_value or int(defaults['port'])),
            'newline': bool(self.get_parameter('newline').get_parameter_value().bool_value if self.get_parameter('newline').get_parameter_value().type in (1, 2) else bool(defaults.get('newline', True))),
            'connect_timeout': float(self.get_parameter('connect_timeout').value),
            'send_timeout': float(self.get_parameter('send_timeout').value),
            'cooldown': float(self.get_parameter('cooldown').value),
        }

        # Mapping arrives as JSON string; fall back to defaults on parse error
        mapping_raw = self.get_parameter('mapping_json').get_parameter_value().string_value
        try:
            parsed = json.loads(mapping_raw) if mapping_raw else {}
            if isinstance(parsed, dict):
                cfg['mapping'] = parsed
            else:
                cfg['mapping'] = defaults.get('mapping', {})
        except Exception:
            cfg['mapping'] = defaults.get('mapping', {})

        # Create bridge with combined params
        self.bridge: ButtonCommandBridge = ButtonCommandBridge(
            host=str(cfg['host']),
            port=int(cfg['port']),
            mapping=dict(cfg['mapping']),
            newline=bool(cfg['newline']),
            connect_timeout=float(cfg['connect_timeout']),
            send_timeout=float(cfg['send_timeout']),
            cooldown=float(cfg['cooldown']),
        )

        self._press_threshold: float = float(self.get_parameter('press_threshold').value)
        topic = self.get_parameter('actions_topic').value
        self.sub = self.create_subscription(String, topic, self._on_actions_msg, 10)
        self.get_logger().info(f"ButtonCommandBridge node started. Subscribed to '{topic}' -> {cfg['host']}:{cfg['port']}")

    def _on_actions_msg(self, msg: String) -> None:
        try:
            data = msg.data
            actions: Dict[str, Any] = json.loads(data) if data else {}
        except Exception as e:
            self.get_logger().warning(f"Invalid actions JSON: {e}")
            return

        try:
            status = self._convert_actions_to_status(actions)
            self.bridge.process_status(status)
        except Exception as e:
            self.get_logger().warning(f"Failed to process actions: {e}")

    def _convert_actions_to_status(self, actions: Dict[str, Any]) -> Dict[str, Dict[str, Any]]:
        """
        Convert VR actions dict (with nested 'buttons') to the bridge's
        expected status format: { name: { 'pressed': bool, 'value': float } }.
        """
        status: Dict[str, Dict[str, Any]] = {}
        buttons = actions.get('buttons', {}) if isinstance(actions, dict) else {}

        def _normalize(v: Any) -> Tuple[bool, float]:
            # Support bool, number, single-element arrays
            try:
                if isinstance(v, bool):
                    return bool(v), 1.0 if v else 0.0
                if isinstance(v, (int, float)):
                    valf = float(v)
                    return (valf > self._press_threshold), valf
                if isinstance(v, (list, tuple)) and len(v) > 0:
                    valf = float(v[0])
                    return (valf > self._press_threshold), valf
            except Exception:
                pass
            return False, 0.0

        if isinstance(buttons, dict):
            for name, v in buttons.items():
                pressed, valf = _normalize(v)
                status[name] = { 'pressed': pressed, 'value': valf }

        return status


def main() -> None:
    rclpy.init()
    node = ButtonCommandBridgeNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
