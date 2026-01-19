from __future__ import annotations

from typing import Dict, Any, Optional

from .bridge import ButtonCommandBridge


# Package-owned defaults so callers don't hard-code in their apps
DEFAULT_CONFIG: Dict[str, Any] = {
    "host": "192.168.57.111",
    "port": 5001,
    "newline": True,
    "connect_timeout": 1.5,
    "send_timeout": 1.5,
    "cooldown": 0.05,
    "mapping": {
        # Map VR buttons to tool commands
        # Adjust here to change package-level behavior
        "X": {"on_press": "mediumNiScrew", "on_release": "STOPSCREW"},
        "Y": {"on_press": "mediumShunScrew", "on_release": "STOPSCREW"},
        # "A": {"on_press": "startDrill", "on_release": "stopDrill"},
        # "B": {"on_press": "start", "on_release": "stop"},
    },
}


def get_default_config() -> Dict[str, Any]:
    # Return a shallow copy so callers don't mutate our module constants
    cfg = dict(DEFAULT_CONFIG)
    # mapping is nested; copy it too
    cfg["mapping"] = dict(DEFAULT_CONFIG.get("mapping", {}))
    return cfg


def create_default_bridge(overrides: Optional[Dict[str, Any]] = None) -> ButtonCommandBridge:
    """
    Create a ButtonCommandBridge using package-owned defaults, with optional overrides.

    Example:
        bridge = create_default_bridge({"host": "192.168.0.10"})
    """
    cfg = get_default_config()
    if overrides:
        # Apply overrides shallowly; mapping can also be overridden fully
        for k, v in overrides.items():
            cfg[k] = v

    return ButtonCommandBridge(
        host=str(cfg.get("host")),
        port=int(cfg.get("port")),
        mapping=dict(cfg.get("mapping", {})),
        newline=bool(cfg.get("newline", True)),
        connect_timeout=float(cfg.get("connect_timeout", 1.5)),
        send_timeout=float(cfg.get("send_timeout", 1.5)),
        cooldown=float(cfg.get("cooldown", 0.05)),
    )
