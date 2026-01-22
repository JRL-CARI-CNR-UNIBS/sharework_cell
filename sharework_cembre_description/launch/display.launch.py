from launch import LaunchDescription
from launch.actions import OpaqueFunction, IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import (
    PathJoinSubstitution, LaunchConfiguration, FindExecutable, Command, ThisLaunchFileDir,
    PythonExpression,
)
from launch.conditions import IfCondition
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch_ros.parameter_descriptions import ParameterValue



def launch_setup(context, *args, **kwargs):
    rviz_config_file = PathJoinSubstitution(
        [
            FindPackageShare('sharework_cembre_description'),
            "config",
            "view_cell.rviz"
        ]
    )

    robot_description_content = ParameterValue(
        Command(
            [
                PathJoinSubstitution([FindExecutable(name='xacro')]),
                " ",
                PathJoinSubstitution(
                    [
                        FindPackageShare('sharework_cembre_description'),
                        "urdf",
                        "sharework_cell.urdf.xacro",
                    ]
                ),
                " ",
                "generate_ros2_control", ":=False",
                " ",
                "robotiq_include_ros2_control", ":=False",
                " ",
                "fake_cameras:=", LaunchConfiguration("fake_cameras"),
                " ",
                "zed_camera_name:=", LaunchConfiguration("zed_camera_name"),
                " ",
                "zed_camera_model:=", LaunchConfiguration("zed_camera_model"),
            ]
        ),
        value_type=str
    )

    robot_description = {'robot_description': robot_description_content}

    joint_target_publisher = Node(
        name='joint_target_publisher',
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        parameters=[{'rate': 30}],
    )

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[robot_description],
    )

    rviz = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        arguments=["-d", rviz_config_file],
        parameters=[robot_description],
    )

    return [
        joint_target_publisher,
        robot_state_publisher,
        rviz
    ]


def generate_launch_description():
    zed_config_path = PathJoinSubstitution(
        [
            FindPackageShare('sharework_cembre_description'),
            'config',
            'zed_params.yaml'
        ]
    )

    # New top-level flag: launch ALL cameras (and their TF) or not.
    launch_arguments = [
        DeclareLaunchArgument(
            'fake_cameras',
            default_value='false',
            description='If true, include camera frames in URDF and launch camera-related nodes.'
        ),

        DeclareLaunchArgument('enable_zed_camera', default_value='true', description='Enable ZED camera launch'),
        DeclareLaunchArgument('enable_dual_realsense', default_value='true', description='Enable dual RealSense camera launch'),
        DeclareLaunchArgument('zed_camera_name', default_value='zed', description='Name of the ZED camera'),
        DeclareLaunchArgument('zed_camera_model', default_value='zed2', description='Model of the ZED camera'),
        DeclareLaunchArgument(
            'zed_camera_config_path',
            default_value=zed_config_path,
            description='Path to the ZED camera config file'
        )
    ]

    # Composite conditions: camera include runs only if launch_cameras AND the specific enable flag.
    zed_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([ThisLaunchFileDir(), '/zed_camera.launch.py']),
        launch_arguments={
            'camera_name': LaunchConfiguration('zed_camera_name'),
            'camera_model': LaunchConfiguration('zed_camera_model'),
            'config_path': LaunchConfiguration('zed_camera_config_path')
        }.items(),
        condition=IfCondition(
            PythonExpression([
                "'", LaunchConfiguration('fake_cameras'), "' == 'fasle' and '",
                LaunchConfiguration('enable_zed_camera'), "' == 'true'"
            ])
        )
    )

    dual_realsense_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([ThisLaunchFileDir(), '/dual_realsense.launch.py']),
        condition=IfCondition(
            PythonExpression([
                "'", LaunchConfiguration('fake_cameras'), "' == 'false' and '",
                LaunchConfiguration('enable_dual_realsense'), "' == 'true'"
            ])
        )
    )


    cameras_tf_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([ThisLaunchFileDir(), '/cameras_tf.launch.py']),
        condition=IfCondition(LaunchConfiguration('fake_cameras'))
    )

    return LaunchDescription(
        launch_arguments + [
            zed_launch,
            dual_realsense_launch,
            cameras_tf_launch,
            OpaqueFunction(function=launch_setup),
        ]
    )

