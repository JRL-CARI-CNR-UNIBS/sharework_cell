from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='world_to_cam_rs1',
            arguments=['0.845205', '1.15052', '2.57347', '0.292609', '0.252083', '-0.649499', '0.654969', 'world', 'rs1_cam_link', '100']
        ),
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='world_to_cam_rs2',
            arguments=['0.649999', '-1.16584', '2.63428', '-0.294127', '0.265496', '0.63995', '0.658381', 'world', 'rs2_cam_link', '100']
        ),
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='world_to_cam_zed',
            arguments=['0.100575', '-0.9304', '2.31042', '0.180663', '0.516604', '0.119341', '0.828395', 'world', 'zed_cam_link', '100']
        ),
    ])