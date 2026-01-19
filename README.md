# Auto Control Workspace (Halfauto) — ROS 2

A ROS 2 workspace for semi‑autonomous dual‑arm robot control using VR (Oculus) inputs, with an optional button→command bridge. It contains multiple Python packages wired together via a launch file for end‑to‑end bringup.

## Overview
- `vr_oculus2_pkg`: Publishes VR controller actions (e.g., from Oculus Quest) into ROS 2.
- `halfauto_pkg`: Dual‑robot controller subscribing to VR actions and commanding two robots (Fairino SDK).
- `button_command_bridge`: Optional node to forward VR button events to external tools over TCP.
- Additional `fairino_*` and config packages are included for hardware/MoveIt descriptions.

## Requirements
- Linux + ROS 2 (e.g., Humble/Foxy). Ensure your ROS 2 environment is sourced.
- Python 3.8+ with standard ROS 2 Python tooling.
- Network connectivity to both robots (defaults: `192.168.57.3` and `192.168.57.2`).
- For VR input, see `src/vr_oculus2_pkg/oculus_reader/README.md` for device setup and data source.

## Build
```bash
# From the workspace root
source /opt/ros/$ROS_DISTRO/setup.bash
colcon build
source install/setup.bash
```

when false, running:

```bash
export PYTHONPATH=$PYTHONPATH:/home/hkust/anaconda3/envs/teleop2/lib/python3.10/site-packages
```

## Quick Start (All‑in‑one Launch)
Launches VR publisher, the dual‑robot controller, and (optionally) the button bridge.
```bash
ros2 launch halfauto_pkg bringup.launch.py \
  robot_ip:=192.168.57.3 \
  robot2_ip:=192.168.57.2 \
  mapping:=delta_base \
  cmd_period:=0.008 \
  joint6_step:=-5.0 \
  start_bridge:=true
```

### Launch Arguments
- `robot_ip`: Right robot IP (default `192.168.57.3`).
- `robot2_ip`: Left robot IP (default `192.168.57.2`).
- `mapping`: `absolute` | `delta_base` | `delta_tool` (default `delta_base`).
- `cmd_period`: Control period seconds for robot commands (default `0.008`).
- `joint6_step`: Degrees to apply to joint‑6 on A‑press (default `-5.0`).
- `start_bridge`: `true|false` to run `button_command_bridge` (default `true`).

Notes:
- The controller enables continuous gripper handling when `--use-gripper` is set (enabled by default in the launch).
- The controller subscribes to `/vr/actions` and publishes TCP poses on `/robot1/tcp_pose` and `/robot2/tcp_pose`.

## Run Nodes Individually
In separate terminals (after `source install/setup.bash`):
```bash
# VR actions publisher
ros2 run vr_oculus2_pkg VR

# Dual‑robot controller
ros2 run halfauto_pkg halfauto_node -- \
  --robot-ip 192.168.57.3 \
  --robot2-ip 192.168.57.2 \
  --vr-mode relative \
  --mapping delta_base \
  --pos-gain 25.0 25.0 25.0 3.0 3.0 3.0 \
  --cmd-period 0.008 \
  --use-gripper \
  --joint6-step -5.0

# Optional button→command bridge
ros2 run button_command_bridge button_command_bridge_node
```

## Topics (Common)
- Input: `/vr/actions` (`std_msgs/String` JSON) — VR action bundle consumed by the controller.
- Output: `/robot1/tcp_pose`, `/robot2/tcp_pose` (`std_msgs/Float64MultiArray`) — current TCP poses.
- Additional inputs (if available in your system): `/depth_to_pose/base1/euler_deg`, `/depth_to_pose/base2/euler_deg` (`geometry_msgs/Vector3Stamped`).

List active topics at runtime:
```bash
ros2 topic list
```

## Logs & Debugging
- Tail node logs:
```bash
# Example: controller stdout
mkdir -p ~/.ros/log
tail -f ~/.ros/log/latest/*halfauto_node*/stdout.log
```
- Verify node graph and topics:
```bash
ros2 node list
ros2 topic list
ros2 topic echo /robot1/tcp_pose
```
- Connection issues: check robot IP reachability (`ping`), firewall settings, and SDK availability.

## Repository Layout
```
src/
  halfauto_pkg/           # Dual‑robot controller + launch
  vr_oculus2_pkg/         # VR actions publisher
  button_command_bridge/  # Optional bridge for button→tool commands
  ...
```

## Contributing
- Use standard ROS 2 Python style and linters.
- Build with `colcon build`, test with `colcon test` when tests are available.
- Open issues/PRs with clear repro steps and logs.

## License
Licensing is per‑package. See each package’s `package.xml` (e.g., `button_command_bridge` is MIT; others may differ).

## Acknowledgements
- Fairino SDK for robot control.
- Oculus/VR input via `vr_oculus2_pkg` and its reader utilities.