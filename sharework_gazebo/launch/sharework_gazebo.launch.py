"""The Sharework cell in Gazebo (gz sim), with MoveIt and RViz.

    ros2 launch sharework_gazebo sharework_gazebo.launch.py

Same robot_description as the real cell (sharework_cembre_description), with
the UR10e on sharework_gazebo/GravityCompensatedGazeboSystem (effort commands
net of gravity, as the UR direct torque control) and the gripper on
gz_ros2_control. Controllers: config/gz_controllers.yaml.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    AppendEnvironmentVariable,
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    OpaqueFunction,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from moveit_configs_utils import MoveItConfigsBuilder

# Packages whose meshes the description loads with package://. Gazebo finds
# package://<pkg>/... under GZ_SIM_RESOURCE_PATH, so it needs their parents.
MESH_PACKAGES = ["sharework_cembre_description", "ur_description", "robotiq_description"]


def is_true(context, name):
    return LaunchConfiguration(name).perform(context).lower() in ("true", "1")


def launch_setup(context):
    share = get_package_share_directory("sharework_gazebo")
    moveit_config_package = "sharework_cembre_moveit_config"
    moveit_share = get_package_share_directory(moveit_config_package)
    use_sim_time = {"use_sim_time": True}

    xacro_mappings = {
        "sim_cameras": LaunchConfiguration("sim_cameras").perform(context),
        "gravity_compensation": LaunchConfiguration("gravity_compensation").perform(context),
        "net_effort_state": LaunchConfiguration("net_effort_state").perform(context),
        "simulation_controllers": os.path.join(share, "config", "gz_controllers.yaml"),
    }

    moveit_config = (
        MoveItConfigsBuilder("sharework_cell", package_name=moveit_config_package)
        .robot_description(
            file_path=LaunchConfiguration("description_file").perform(context),
            mappings=xacro_mappings,
        )
        .robot_description_semantic(
            file_path=os.path.join(moveit_share, "config", "sharework_cell.srdf"))
        .planning_scene_monitor(
            publish_robot_description=False,
            publish_robot_description_semantic=True,
            publish_planning_scene=True,
        )
        .planning_pipelines(
            default_planning_pipeline="ompl",
            pipelines=["ompl", "chomp", "pilz_industrial_motion_planner"],
        )
        .pilz_cartesian_limits(
            file_path=os.path.join(moveit_share, "config", "pilz_cartesian_limits.yaml"))
        .joint_limits(file_path=os.path.join(moveit_share, "config", "joint_limits.yaml"))
        .trajectory_execution(file_path=os.path.join(share, "config", "moveit_controllers.yaml"))
        .robot_description_kinematics()
        .to_moveit_configs()
    )

    gz_args = "-r -v 3 " + LaunchConfiguration("world").perform(context)
    if not is_true(context, "gui"):
        gz_args += " -s --headless-rendering"
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([FindPackageShare("ros_gz_sim"), "launch", "gz_sim.launch.py"])),
        launch_arguments={"gz_args": gz_args, "on_exit_shutdown": "true"}.items(),
    )

    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="both",
        parameters=[moveit_config.robot_description, use_sim_time],
    )

    spawn = Node(
        package="ros_gz_sim",
        executable="create",
        output="screen",
        arguments=["-topic", "robot_description", "-name", "sharework_cell",
                   "-allow_renaming", "false"],
    )

    bridge_file = "gz_bridge_cameras.yaml" if is_true(context, "sim_cameras") else "gz_bridge.yaml"
    bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        output="screen",
        parameters=[{"config_file": os.path.join(share, "config", bridge_file)}, use_sim_time],
    )

    # Gazebo's clouds are in the camera link frame, not the optical one.
    points_relay = Node(
        package="sharework_gazebo",
        executable="points_frame_relay",
        output="screen",
        parameters=[{"cameras": ["zed", "rs1", "rs2"]}, use_sim_time],
        condition=IfCondition(LaunchConfiguration("sim_cameras")),
    )

    def spawner(controllers, active=True):
        return Node(
            package="controller_manager",
            executable="spawner",
            output="screen",
            arguments=controllers
            + ["--controller-manager", "/controller_manager",
               "--controller-manager-timeout", "120"]
            + ([] if active else ["--inactive"]),
            parameters=[use_sim_time],
        )

    arm_controllers = [
        "joint_trajectory_controller",
        "forward_position_controller",
        "forward_velocity_controller",
        "forward_effort_controller",
    ]
    initial = LaunchConfiguration("initial_joint_controller").perform(context)
    if initial not in arm_controllers:
        raise RuntimeError(f"initial_joint_controller must be one of {arm_controllers}")
    spawners = [
        spawner(["joint_state_broadcaster",
                 "robotiq_gripper_controller",
                 "robotiq_gripper_status_broadcaster",
                 "robotiq_normalized_position_broadcaster",
                 initial]),
        spawner([c for c in arm_controllers if c != initial], active=False),
    ]

    move_group = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=[moveit_config.to_dict(), use_sim_time],
        condition=IfCondition(LaunchConfiguration("launch_moveit")),
    )

    rviz = Node(
        package="rviz2",
        executable="rviz2",
        output="log",
        arguments=["-d", os.path.join(moveit_share, "config", "moveit.rviz")],
        parameters=[
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.planning_pipelines,
            moveit_config.robot_description_kinematics,
            use_sim_time,
        ],
        condition=IfCondition(LaunchConfiguration("launch_rviz")),
    )

    return [gazebo, robot_state_publisher, spawn, bridge, points_relay, move_group, rviz] + spawners


def generate_launch_description():
    resource_paths = [
        AppendEnvironmentVariable(
            "GZ_SIM_RESOURCE_PATH",
            os.path.dirname(get_package_share_directory(pkg)),
        )
        for pkg in MESH_PACKAGES
    ]
    args = [
        DeclareLaunchArgument("gui", default_value="true",
                              description="Open the Gazebo window (false: server only)."),
        DeclareLaunchArgument("launch_rviz", default_value="true", description="Start RViz."),
        DeclareLaunchArgument("launch_moveit", default_value="true",
                              description="Start MoveIt's move_group."),
        DeclareLaunchArgument("sim_cameras", default_value="true",
                              description="Render the ZED and the two RealSense in Gazebo."),
        DeclareLaunchArgument(
            "gravity_compensation", default_value="true",
            description="Effort commands net of gravity, as the UR direct torque control."),
        DeclareLaunchArgument(
            "net_effort_state", default_value="false",
            description="Effort state net of gravity too (false: the torque Gazebo applies)."),
        DeclareLaunchArgument(
            "initial_joint_controller", default_value="joint_trajectory_controller",
            description="Arm controller to activate: joint_trajectory_controller, "
            "forward_position_controller, forward_velocity_controller or "
            "forward_effort_controller. The others are loaded inactive."),
        DeclareLaunchArgument(
            "description_file",
            default_value=os.path.join(
                get_package_share_directory("sharework_gazebo"), "urdf", "sharework_cell_gz.urdf.xacro"),
            description="Robot description xacro. It gets the sim_cameras, gravity_compensation, "
            "net_effort_state and simulation_controllers mappings."),
        DeclareLaunchArgument(
            "world",
            default_value=os.path.join(
                get_package_share_directory("sharework_gazebo"), "worlds", "sharework_cell.sdf"),
            description="Gazebo world file."),
    ]
    return LaunchDescription(resource_paths + args + [OpaqueFunction(function=launch_setup)])
