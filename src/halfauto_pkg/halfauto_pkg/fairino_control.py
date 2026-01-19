import logging
import time
from typing import Sequence, Optional, Dict, Any
import argparse

try:
    from fairino import Robot as FairinoSDK
except ImportError:
    FairinoSDK = None

# 尝试导入 VR 控制器
try:
    from vr_oculus import OculusQuest3Controller, VRConfig
except Exception:
    OculusQuest3Controller = None  # 若无需 VR 可忽略
    VRConfig = None

logger = logging.getLogger(__name__)
logging.basicConfig(level=logging.INFO)


class FairinoConfig:
    def __init__(
        self,
        robot_ip: str,
        cmd_period: float = 0.008,  # ServoCart 调用周期 (s)
        pos_gain: Optional[Sequence[float]] = None,  # 增量模式 6 轴增益
    ):
        self.robot_ip = robot_ip
        self.cmd_period = cmd_period
        self.pos_gain = list(pos_gain) if pos_gain else [1.0] * 6


class FairinoCartRobot:
    """
    简化笛卡尔伺服封装
      mode=0 绝对
      mode=1 基坐标增量
      mode=2 工具坐标增量
    """

    name = "FairinoCartSimple"

    def __init__(self, config: FairinoConfig):
        # super().__init__(config)  # 无父类，去掉
        self.config = config
        self.robot_ip = config.robot_ip
        self._cmdT = config.cmd_period
        self._pos_gain = config.pos_gain
        self.robot = None
        self._last_abs_pose: Optional[list[float]] = None
        self._last_send_time = 0.0

        # 可调参数
        self._acc = 0.0
        self._vel = 0.0
        self._filterT = 0.0
        self._gain_amplifier = 0.0

    # ---------------- 连接 ----------------
    @property
    def is_connected(self) -> bool:
        return self.robot is not None

    def connect(self):
        if self.is_connected:
            return
        if FairinoSDK is None:
            raise ImportError("未找到 fairino SDK，请先安装。")
        try:
            self.robot = FairinoSDK.RPC(self.robot_ip) if hasattr(FairinoSDK, "RPC") else FairinoSDK(self.robot_ip)
            try:
                self.robot.SetSpeed(20)
            except Exception:
                pass
            logger.info(f"[Fairino] Connected {self.robot_ip}")
        except Exception as e:
            self.robot = None
            raise RuntimeError(f"连接失败 {self.robot_ip}: {e}") from e

    def disconnect(self):
        if not self.is_connected:
            return
        try:
            if hasattr(self.robot, "CloseRPC"):
                self.robot.CloseRPC()
        finally:
            self.robot = None
            logger.info("[Fairino] Disconnected")

    # ---------------- 内部工具 ----------------
    def _throttle(self):
        now = time.time()
        dt = now - self._last_send_time
        remain = self._cmdT - dt
        if remain > 0:
            time.sleep(remain)
        self._last_send_time = time.time()

    def _call_servo(self, mode: int, desc_pos: Sequence[float]):
        if not self.is_connected:
            raise RuntimeError("Robot 未连接")
        try:
            self.robot.ServoCart(
                mode=mode,
                desc_pos=list(desc_pos),
                pos_gain=self._pos_gain,
                acc=self._acc,
                vel=self._vel,
                cmdT=self._cmdT,
                filterT=self._filterT,
                gain=self._gain_amplifier,
            )
        except Exception as e:
            logger.error(f"ServoCart 失败 mode={mode}: {e}")

    # ---------------- API: 绝对 / 增量 ----------------
    def send_abs_pose(self, pose6: Sequence[float]):
        if len(pose6) != 6:
            raise ValueError("pose6 需要 6 个元素")
        self._call_servo(mode=0, desc_pos=pose6)
        self._last_abs_pose = list(pose6)
        self._throttle()

    def send_delta(self, delta6: Sequence[float]):
        if len(delta6) != 6:
            raise ValueError("delta6 需要 6 个元素")
        self._call_servo(mode=1, desc_pos=delta6)
        if self._last_abs_pose:
            self._last_abs_pose = [a + d for a, d in zip(self._last_abs_pose, delta6)]
        self._throttle()

    def send_delta_tool(self, delta6: Sequence[float]):
        if len(delta6) != 6:
            raise ValueError("delta6 需要 6 个元素")
        self._call_servo(mode=2, desc_pos=delta6)
        if self._last_abs_pose:
            self._last_abs_pose = [a + d for a, d in zip(self._last_abs_pose, delta6)]
        self._throttle()

    # ---------------- Oculus 映射 ----------------
    def apply_vr_pose(
        self,
        vr_pose6: Sequence[float],
        *,
        mode: str = "absolute",
        scale_pos: float = 1.0,
        scale_rot: float = 1.0,
        origin_offset: Sequence[float] = (0, 0, 0),
        rot_offset: Sequence[float] = (0, 0, 0),
    ):
        if len(vr_pose6) != 6:
            raise ValueError("vr_pose6 需要 6 个元素")
        x, y, z, rx, ry, rz = vr_pose6
        if mode == "absolute":
            pose = [
                origin_offset[0] + x * scale_pos,
                origin_offset[1] + y * scale_pos,
                origin_offset[2] + z * scale_pos,
                rot_offset[0] + rx * scale_rot,
                rot_offset[1] + ry * scale_rot,
                rot_offset[2] + rz * scale_rot,
            ]
            self.send_abs_pose(pose)
        elif mode == "delta_base":
            self.send_delta([
                x * scale_pos, y * scale_pos, z * scale_pos,
                rx * scale_rot, ry * scale_rot, rz * scale_rot
            ])
        elif mode == "delta_tool":
            self.send_delta_tool([
                x * scale_pos, y * scale_pos, z * scale_pos,
                rx * scale_rot, ry * scale_rot, rz * scale_rot
            ])
        else:
            raise ValueError("mode 只能为 absolute | delta_base | delta_tool")

    # ---------------- 直接执行 VR 动作 dict ----------------
    def apply_action_dict(self, action: Dict[str, Any], mapping: str):
        """
        mapping:
          absolute   : 使用 abs_* 键 -> send_abs_pose
          delta_base : 使用 delta_* -> send_delta
          delta_tool : 使用 delta_* -> send_delta_tool
        """
        if mapping == "absolute":
            keys = ["abs_x", "abs_y", "abs_z", "abs_roll", "abs_pitch", "abs_yaw"]
            if all(k in action for k in keys):
                self.send_abs_pose([action[k] for k in keys])
        elif mapping == "delta_base":
            keys = ["delta_x", "delta_y", "delta_z", "delta_roll", "delta_pitch", "delta_yaw"]
            if all(k in action for k in keys):
                self.send_delta([action[k] for k in keys])
        elif mapping == "delta_tool":
            keys = ["delta_x", "delta_y", "delta_z", "delta_roll", "delta_pitch", "delta_yaw"]
            if all(k in action for k in keys):
                self.send_delta_tool([action[k] for k in keys])
        else:
            raise ValueError("mapping 必须为 absolute | delta_base | delta_tool")

    # ---------------- 观测 (最小) ----------------
    def get_current_pose(self) -> list[float]:
        if not self.is_connected:
            return self._last_abs_pose or [0.0] * 6
        try:
            if hasattr(self.robot, "GetActualTCPPose"):
                pose = self.robot.GetActualTCPPose()
                if pose and len(pose) >= 6:
                    self._last_abs_pose = list(pose[:6])
            return self._last_abs_pose or [0.0] * 6
        except Exception:
            return self._last_abs_pose or [0.0] * 6

    # ---------------- 动态参数 ----------------
    def configure(self, **kw):
        if "cmd_period" in kw:
            self._cmdT = float(kw["cmd_period"])
        if "pos_gain" in kw:
            pg = kw["pos_gain"]
            if len(pg) == 6:
                self._pos_gain = list(map(float, pg))
        for k in ("acc", "vel", "filterT", "gain"):
            if k in kw:
                setattr(self, f"_{k if k!='gain' else 'gain_amplifier'}", float(kw[k]))
        logger.info(f"配置更新: {kw}")

    # ---------------- VR 主循环 ----------------
    def run_vr_loop(
        self,
        *,
        vr_mode: str = "relative",
        mapping: str = "delta_base",
        robot_idle_sleep: float = 0.01,
        print_interval: float = 1.0,
    ):
        """
        vr_mode: relative | absolute (对应 VRConfig.control_mode)
        mapping: absolute | delta_base | delta_tool (机器人执行方式)
        """
        if OculusQuest3Controller is None:
            raise RuntimeError("未找到 VR 控制器类 (vr_oculus 导入失败)")
        vr = OculusQuest3Controller(
            VRConfig(control_mode=vr_mode)
        )
        vr.connect()
        logger.info(f"[VR] Connected, vr_mode={vr_mode}, mapping={mapping}")
        last_print = time.time()
        try:
            while True:
                act = vr.get_action()
                self.apply_action_dict(act, mapping=mapping)
                now = time.time()
                if now - last_print > print_interval:
                    logger.info(f"Act: {{k: act[k] for k in list(act)[:7]}}")
                    last_print = now
                # 若离合未按(全 0)，避免高频空发送
                if all(v == 0.0 for k, v in act.items() if k.startswith("delta_") or k.startswith("abs_")):
                    time.sleep(robot_idle_sleep)
        except KeyboardInterrupt:
            logger.info("用户中断 VR loop")
        finally:
            vr.disconnect()

    # ---------------- 退出 ----------------
    def __del__(self):
        try:
            self.disconnect()
        except Exception:
            pass


# ---------------- 脚本入口 ----------------
def _build_arg_parser():
    p = argparse.ArgumentParser(description="Fairino + Oculus VR 控制")
    p.add_argument("--robot-ip", required=True, help="机器人 IP")
    p.add_argument("--vr-mode", default="relative", choices=["relative", "absolute"], help="VR 输出模式")
    p.add_argument("--mapping", default="delta_base", choices=["absolute", "delta_base", "delta_tool"], help="机器人执行映射")
    p.add_argument("--pos-gain", type=float, nargs=6, default=None, help="六轴增量增益")
    p.add_argument("--cmd-period", type=float, default=0.008, help="发送周期")
    return p


if __name__ == "__main__":
    parser = _build_arg_parser()
    args = parser.parse_args()

    cfg = FairinoConfig(
        robot_ip=args.robot_ip,
        cmd_period=args.cmd_period,
        pos_gain=args.pos_gain,
    )
    robot = FairinoCartRobot(cfg)
    robot.connect()
    try:
        robot.run_vr_loop(vr_mode=args.vr_mode, mapping=args.mapping)
    finally:
        robot.disconnect()