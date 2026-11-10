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
sudo apt install ros-jazzy-ur ros-jazzy-moveit-ros-planning-interface ros-jazzy-geometric-shapes

```

---

# Installation

Clone this repository together with its required dependencies into the `src` folder of your ROS 2 workspace.

```bash
cd ~/ros2_ws/src

# Robotiq gripper: official driver and controllers, plus the URCap socket plugin
git clone https://github.com/robotiq/robotiq-ros.git
touch robotiq-ros/robotiq_tsf/COLCON_IGNORE robotiq-ros/robotiq_tactile_sdk_vendor/COLCON_IGNORE  # tactile sensors, not needed
git clone https://github.com/JRL-CARI-CNR-UNIBS/robotiq_socket_driver.git

git clone -b jazzy https://github.com/JRL-CARI-CNR-UNIBS/sharework_cell.git
```

Install the dependencies of robotiq-ros:

```bash
cd ~/ros2_ws
rosdep install --from-paths src/robotiq-ros src/robotiq_socket_driver --ignore-src -y
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
| `robotiq_use_socket_communication` | `true`: drive the gripper through the Robotiq Gripper URCap socket (`robotiq_socket_driver`, `robotiq_ip_address`:`robotiq_port`). `false`: through the UR tool communication serial port (`robotiq_driver`, needs `use_tool_communication:=true`). Must be explicitly set. |
| `enable_zed_camera` | Enables the ZED stereo camera driver. Must be explicitly set (`true` or `false`). |
| `enable_dual_realsense` | Enables dual Intel RealSense camera setup. Must be explicitly set (`true` or `false`). |
| `launch_rviz` | Starts RViz visualization. Must be explicitly set (`true` or `false`). |
| `robotiq_activate_gripper_by_default` | Socket only. `1`: reset and activate the gripper at startup (the fingers sweep, a held object is dropped). `0`: activate it only if it is not activated yet, keeping the current grip. |
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
    enable_wrist_camera:=false
```


---

# Robotiq Gripper

The gripper uses the controllers of [robotiq-ros](https://github.com/robotiq/robotiq-ros):

| Controller | Interface |
|---|---|
| `robotiq_gripper_controller` | action `/robotiq_gripper_controller/gripper_cmd`, `control_msgs/action/ParallelGripperCommand` |
| `robotiq_gripper_status_broadcaster` | topic `/robotiq_gripper_status_broadcaster/status`, `robotiq_msgs/GripperStatus` |
| `robotiq_activation_controller` | service `/robotiq_activation_controller/reactivate_gripper`, `std_srvs/srv/Trigger` |
| `robotiq_normalized_position_broadcaster` | topic `/robotiq_normalized_position_broadcaster/joint_states`, `sensor_msgs/JointState`: joint `robotiq_normalized_joint`, position from 0 (open) to 1 (closed). Moved off `/joint_states`, which carries a single message with all the joints |

Close the gripper (joint position in rad, 0 open, 0.7929 closed):

```bash
ros2 action send_goal /robotiq_gripper_controller/gripper_cmd control_msgs/action/ParallelGripperCommand \
  "{command: {name: [ur10e_robotiq_85_left_knuckle_joint], position: [0.79]}}"
```

On the real gripper the goal is decided by its object detection: it succeeds
when the fingers reach the position or stop on an object. The result tells which
(`reached_goal` or `stalled`), and `object_detection` on the status topic is
`2` when an object was caught while closing, `1` while opening, `3` when the
fingers reached the position with nothing in between, `0` while moving.
`velocity` and `effort` in the goal set the gripper's speed (m/s) and force (N).

This replaces the old `robotiq_action_controller`
(`control_msgs/action/GripperCommand` on `/robotiq_action_controller/gripper_cmd`)
and `robotiq_forward_command_controller`: clients have to move to the new
action name and type.

With `fake_gripper:=true` the gripper is the ros2_control mock: the goal
succeeds when the position is reached, the status topic is silent and the
reactivation service never answers.

To try the socket path without the robot, run the simulated URCap and point the
cell at it:

```bash
ros2 run robotiq_socket_driver fake_urcap_server --port 63352 --object 120
ros2 launch sharework_cembre_bringup sharework_cembre_bringup.launch.py \
    fake_ur:=true fake_gripper:=false \
    robotiq_use_socket_communication:=true robotiq_ip_address:=127.0.0.1 \
    robotiq_activate_gripper_by_default:=1 \
    enable_zed_camera:=false enable_dual_realsense:=false enable_wrist_camera:=false launch_rviz:=true
```


---

# Gazebo Simulation

`sharework_gazebo` runs the cell in Gazebo Harmonic (gz sim), with MoveIt and
RViz. It uses the same description as the real cell: the UR10e and the
Robotiq 2F-85 go through `gz_ros2_control`, and the ZED and the two RealSense
cameras are rendered by Gazebo.

Additional dependencies:

```bash
sudo apt install ros-jazzy-ros-gz ros-jazzy-gz-ros2-control ros-jazzy-pinocchio
```

Launch:

```bash
ros2 launch sharework_gazebo sharework_gazebo.launch.py
```

| Argument | Default | Meaning |
|---|---|---|
| `gui` | `true` | Gazebo window (`false`: server only, headless rendering) |
| `launch_rviz` | `true` | RViz with the MoveIt configuration |
| `launch_moveit` | `true` | `move_group` |
| `sim_cameras` | `true` | Render the ZED, `rs1` and `rs2` |
| `initial_joint_controller` | `joint_trajectory_controller` | Arm controller to activate; the others are loaded inactive |
| `gravity_compensation` | `true` | Effort commands net of gravity (see below) |
| `net_effort_state` | `false` | Report the effort state net of gravity as well |
| `world` | `worlds/sharework_cell.sdf` | Gazebo world |
| `description_file` | `urdf/sharework_cell_gz.urdf.xacro` | Robot description xacro (e.g. a wrapper that adds sensors or plugins) |

Controllers (`sharework_gazebo/config/gz_controllers.yaml`):
`joint_trajectory_controller` (used by MoveIt), `forward_position_controller`,
`forward_velocity_controller`, `forward_effort_controller` and the gripper
controllers, with the same action as on the real cell. The UR driver
controllers (scaled JTC, I/O, speed scaling, F/T sensor) do not exist in
simulation, and the gripper has no object detection or reactivation service.

## Torque control

As with the UR direct torque control, the effort command is the torque **net
of gravity**. The hardware plugin `sharework_gazebo/GravityCompensatedGazeboSystem`
adds g(q), computed with Pinocchio on the simulated `robot_description`, before
Gazebo applies it. A zero command holds the arm:

```bash
ros2 control switch_controllers --activate forward_effort_controller --deactivate joint_trajectory_controller
ros2 topic pub /forward_effort_controller/commands std_msgs/msg/Float64MultiArray "{data: [0, 0, 0, 0, 0, 0]}"
```

The simulated joints have no friction, so any velocity the arm has when
switching stays (a few hundredths of rad/s after a switch from the trajectory
controller). Switch from a controller that holds the arm still: one started
directly in effort mode lets the arm fall until it activates. The gripper
joints are taken at their zero position in g(q); the error is negligible.

## Cameras

Topics follow the RealSense driver layout for every camera (`zed`, `rs1`,
`rs2`): `<camera>/color/image_raw`, `<camera>/color/camera_info`,
`<camera>/depth/image_rect_raw` (optical frame) and
`<camera>/depth/color/points` (in `<camera>_camera_link`). They are bridged on
demand. Without a GPU, rendering is slow (about 6 Hz instead of 15): use
`sim_cameras:=false` if the cameras are not needed.

Notes:

- The robot and the cell are one Gazebo model, so the robot does not collide
  with the cell (MoveIt keeps them apart). Objects spawned in the world collide
  with both.
- Gazebo prints `the chosen physics engine does not support mimic constraints`:
  harmless, `gz_ros2_control` drives the gripper's mimic joints itself.

---

# Repository Contents

The repository contains packages for:

- robot bringup
- Gazebo simulation (`sharework_gazebo`)
- robot descriptions (UR10e + Robotiq)
- hardware interfaces
- camera integration
- launch files
- RViz configurations
- Sharework cell configuration

---

# License

See the `LICENSE` file for licensing information.
