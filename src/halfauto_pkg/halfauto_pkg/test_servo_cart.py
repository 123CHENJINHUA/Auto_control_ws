import logging
from fairino_control import FairinoConfig, FairinoCartRobot

# 配置日志，指定编码为 UTF-8
logging.basicConfig(level=logging.INFO, format='%(message)s', encoding='utf-8')
logger = logging.getLogger(__name__)

def main():
    # 配置机器人
    robot_ip = "192.168.58.2"  # 替换为实际机器人 IP
    config = FairinoConfig(robot_ip=robot_ip, cmd_period=0.008, pos_gain=[1.0] * 6)
    robot = FairinoCartRobot(config)

    try:
        # 连接机器人
        robot.connect()
        logger.info("机器人已连接")

        # 手动输入 delta6
        delta6 = [0.00684*20, 0.00231*20, 0.00684*20, 0.0, 0.0, 0.0]  # 示例增量值
        logger.info(f"发送增量: {delta6}")

        # 调用增量运动
        robot.send_delta(delta6)
        logger.info("增量运动完成")

        # 获取当前位姿
        current_pose = robot.robot.GetActualTCPPose()[1]  # 修改为调用封装的 get_current_pose 方法
        logger.info(f"当前位姿: {current_pose}")

    except Exception as e:
        logger.error(f"测试失败: {e}")
    finally:
        # 断开连接
        robot.disconnect()
        logger.info("机器人已断开连接")

if __name__ == "__main__":
    main()
