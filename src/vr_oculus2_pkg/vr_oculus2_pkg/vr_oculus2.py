import time
from dataclasses import dataclass
from typing import Any, Dict, Optional

import numpy as np


from .oculus_reader.reader import OculusReader



@dataclass
class VRConfig:
    control_mode: str = "relative"      # "relative" | "absolute"
    displacement_scale: float = 1.0
    angle_scale: float = 1.0
    use_gripper: bool = False
    # 单独定义左右控制器按钮名称
    right_clutch_button: str = "rightGrip"
    left_clutch_button: str = "leftGrip"
    right_gripper_button: str = "rightTrig"
    left_gripper_button: str = "leftTrig"
    max_lin_step: float = 0.1
    max_ang_step: float = 0.1
    deadband: float = 1e-5
    round_digits: int = 5


class OculusQuest3Controller:
    def __init__(self, config: Optional[VRConfig] = None):
        self.config = config or VRConfig()
        self.reader: Optional[OculusReader] = None
        # 使用字典存储左右两侧的状态
        self.prev_pos: Dict[str, np.ndarray] = {}
        self.prev_rot: Dict[str, np.ndarray] = {}
        self.origin_pos: Dict[str, np.ndarray] = {}
        self.origin_rot: Dict[str, np.ndarray] = {}

    def connect(self):
        if self.reader is None:
            self.reader = OculusReader()

    def disconnect(self):
        self.reader = None

    def is_connected(self) -> bool:
        return self.reader is not None

    def calibrate(self):
        self.origin_pos.clear()
        self.origin_rot.clear()

    def get_action(self) -> Dict[str, Any]:
        if not self.is_connected():
            return self._zero_action()

        transforms, buttons = self.reader.get_transformations_and_buttons()
        buttons = buttons or {}
        sides = {'r': 'right', 'l': 'left'}

        # 结果字典
        action: Dict[str, Any] = {}

        for raw_side, side_name in sides.items():
            if raw_side not in transforms:
                # 缺失该侧控制器，填零
                action.update(self._zero_side(side_name))
                continue

            T = transforms[raw_side]
            pos = T[:3, 3]
            rot_m = T[:3, :3]

            # 按钮名称
            clutch_button = getattr(self.config, f"{side_name}_clutch_button")
            gripper_button = getattr(self.config, f"{side_name}_gripper_button")

            clutch_val = self._get_button(buttons, clutch_button)

            if clutch_val <= 0.5:
                # 释放离合，重置相对参考
                self.prev_pos.pop(side_name, None)
                self.prev_rot.pop(side_name, None)
                # 输出零
                action.update(self._zero_side(side_name))
                continue

            if self.config.control_mode == "relative":
                dx, dy, dz, droll, dpitch, dyaw = self._compute_relative(side_name, pos, rot_m)
                key_prefix = f"{side_name}_delta_"
            else:
                dx, dy, dz, droll, dpitch, dyaw = self._compute_absolute(side_name, pos, rot_m)
                key_prefix = f"{side_name}_abs_"

            grip_raw = self._get_button(buttons, gripper_button)
            gripper = float(grip_raw) if self.config.use_gripper else 0.0

            dx, dy, dz = [self._limit_scale(v, self.config.displacement_scale, self.config.max_lin_step) for v in (dx, dy, dz)]
            droll, dpitch, dyaw = [self._limit_scale(v, self.config.angle_scale, self.config.max_ang_step) for v in (droll, dpitch, dyaw)]

            action[f"{key_prefix}x"] = dx
            action[f"{key_prefix}y"] = dy
            action[f"{key_prefix}z"] = dz
            action[f"{key_prefix}roll"] = droll
            action[f"{key_prefix}pitch"] = dpitch
            action[f"{key_prefix}yaw"] = dyaw
            action[f"{side_name}_gripper"] = gripper

            # 更新上一帧
            self.prev_pos[side_name] = pos
            self.prev_rot[side_name] = rot_m

        # 按钮统一附加
        action["buttons"] = buttons
        return action

    def _compute_relative(self, side: str, pos: np.ndarray, rot_m: np.ndarray):
        if side not in self.prev_pos:
            return 0.0, 0.0, 0.0, 0.0, 0.0, 0.0
        dpos = pos - self.prev_pos[side]
        rpy = self.rotation_matrix_to_euler(rot_m)
        prev_rpy = self.rotation_matrix_to_euler(self.prev_rot[side])
        drpy = [c - p for c, p in zip(rpy, prev_rpy)]
        return (*dpos.tolist(), *drpy)

    def _compute_absolute(self, side: str, pos: np.ndarray, rot_m: np.ndarray):
        if side not in self.origin_pos:
            self.origin_pos[side] = pos.copy()
            self.origin_rot[side] = rot_m.copy()
        dpos = pos - self.origin_pos[side]
        rpy = self.rotation_matrix_to_euler(rot_m)
        base_rpy = self.rotation_matrix_to_euler(self.origin_rot[side])
        drpy = [c - b for c, b in zip(rpy, base_rpy)]
        return (*dpos.tolist(), *drpy)

    def _limit_scale(self, val: float, scale: float, limit_abs: float):
        v = val * scale
        if abs(v) < self.config.deadband:
            v = 0.0
        if v > limit_abs:
            v = limit_abs
        elif v < -limit_abs:
            v = -limit_abs
        return round(v, self.config.round_digits)

    def _zero_side(self, side_name: str) -> Dict[str, float]:
        if self.config.control_mode == "relative":
            base_keys = ["delta_x", "delta_y", "delta_z", "delta_roll", "delta_pitch", "delta_yaw"]
        else:
            base_keys = ["abs_x", "abs_y", "abs_z", "abs_roll", "abs_pitch", "abs_yaw"]
        prefix = f"{side_name}_"
        d = {prefix + k: 0.0 for k in base_keys}
        d[f"{side_name}_gripper"] = 0.0
        return d

    def _zero_action(self):
        action = {}
        action.update(self._zero_side("right"))
        action.update(self._zero_side("left"))
        action["buttons"] = {}
        return action

    @staticmethod
    def _get_button(buttons: Dict[str, Any], name: str) -> float:
        val = buttons.get(name, 0.0)
        if isinstance(val, (tuple, list)):
            val = val[0]
        try:
            return float(val)
        except Exception:
            return 0.0

    @staticmethod
    def rotation_matrix_to_euler(R: np.ndarray):
        sy = np.sqrt(R[0, 0] ** 2 + R[1, 0] ** 2)
        singular = sy < 1e-6
        if not singular:
            roll = np.arctan2(R[2, 1], R[2, 2])
            pitch = np.arctan2(-R[2, 0], sy)
            yaw = np.arctan2(R[1, 0], R[0, 0])
        else:
            roll = np.arctan2(-R[1, 2], R[1, 1])
            pitch = np.arctan2(-R[2, 0], sy)
            yaw = 0.0
        return roll, pitch, yaw


if __name__ == "__main__":
    ctrl = OculusQuest3Controller()
    ctrl.connect()
    print("开始读取 (Ctrl+C 退出)")
    try:
        while True:
            act = ctrl.get_action()
            print(act)
            time.sleep(0.05)
    except KeyboardInterrupt:
        pass
    finally:
        ctrl.disconnect()
