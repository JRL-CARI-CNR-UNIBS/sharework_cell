# Sharework Cell

ROS 2 packages for the **Sharework robotic cell**, including the robot description, hardware interfaces, launch files, and the integration of peripherals such as the Robotiq gripper and cameras.

The repository provides a complete bringup for both real hardware and simulation/testing configurations.

---

# Requirements

- Ubuntu 24.04
- ROS 2 Jazzy

## System Dependencies

Install the Universal Robots ROS 2 driver:

```bash
sudo apt update
sudo apt install ros-jazzy-ur
```

---

# Installation

Clone this repository together with its required dependencies into the `src` folder of your ROS 2 workspace.

```bash
cd ~/ros2_ws/src

git clone -b radians_jazzy https://github.com/JRL-CARI-CNR-UNIBS/ros2_robotiq_gripper.git

git clone -b ros2 https://github.com/tylerjw/serial.git

git clone -b jazzy https://github.com/JRL-CARI-CNR-UNIBS/ur10e_robotiq.git

git clone -b jazzy https://github.com/JRL-CARI-CNR-UNIBS/sharework_cell.git
```

Build the workspace:

```bash
cd ~/ros2_ws

source /opt/ros/jazzy/setup.bash

colcon build --symlink-install

source install/setup.bash
```

---

# Launching the Cell

The complete Sharework cell can be started with:

```bash
ros2 launch sharework_cembre_bringup sharework_cembre_bringup.launch.py \
    fake_ur:=false \
    fake_gripper:=false \
    robotiq_use_socket_communication:=true \
    enable_zed_camera:=false \
    enable_dual_realsense:=false \
    launch_rviz:=true \
    robotiq_activate_gripper_by_default:=1 \
    enable_wrist_camera:=true
```
# Launch Arguments

The bringup launch file requires all arguments to be explicitly provided at launch time.

These parameters define the hardware configuration of the cell and **must not be omitted**.

| Argument | Description |
|----------|-------------|
| `fake_ur` | Enables a simulated UR robot instead of the real hardware. Must be explicitly set to `true` or `false`. |
| `fake_gripper` | Enables a simulated Robotiq gripper instead of the real device. Must be explicitly set to `true` or `false`. |
| `robotiq_use_socket_communication` | Enables socket-based communication with the Robotiq gripper. Must be explicitly set (`true` or `false`). |
| `enable_zed_camera` | Enables the ZED stereo camera driver. Must be explicitly set (`true` or `false`). |
| `enable_dual_realsense` | Enables dual Intel RealSense camera setup. Must be explicitly set (`true` or `false`). |
| `launch_rviz` | Starts RViz visualization. Must be explicitly set (`true` or `false`). |
| `robotiq_activate_gripper_by_default` | Automatically activates the Robotiq gripper at startup (`0` or `1`). |
| `enable_wrist_camera` | Enables the wrist-mounted camera. Must be explicitly set (`true` or `false`). |

---

## Example: Real Hardware Setup

```bash
ros2 launch sharework_cembre_bringup sharework_cembre_bringup.launch.py \
    fake_ur:=false \
    fake_gripper:=false \
    robotiq_use_socket_communication:=true \
    enable_zed_camera:=false \
    enable_dual_realsense:=false \
    launch_rviz:=true \
    robotiq_activate_gripper_by_default:=1 \
    enable_wrist_camera:=true
```

---

## Example: Full Simulation Setup

```bash
ros2 launch sharework_cembre_bringup sharework_cembre_bringup.launch.py \
    fake_ur:=true \
    fake_gripper:=true \
    robotiq_use_socket_communication:=true \
    enable_zed_camera:=false \
    enable_dual_realsense:=false \
    launch_rviz:=true \
    robotiq_activate_gripper_by_default:=1 \
    enable_wrist_camera:=true
```

```

---

# Repository Contents

The repository contains packages for:

- robot bringup
- robot descriptions (UR10e + Robotiq)
- hardware interfaces
- camera integration
- launch files
- RViz configurations
- Sharework cell configuration

---

# License

See the `LICENSE` file for licensing information.
