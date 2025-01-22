from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='world_to_cam_rs1',
            arguments=['--x', '0.845205', '--y', '1.15052', '--z', '2.57347',
                       '--qx', '0.292609', '--qy', '0.252083', '--qz', '-0.649499', '--qw', '0.654969',
                       '--frame-id', 'world', '--child-frame-id', 'rs1_camera_link_calibration_check']
        ),
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='world_to_cam_rs2',
            arguments=['--x', '0.649999', '--y', '-1.16584', '--z', '2.63428',
                       '--qx', '-0.294127', '--qy', '0.265496', '--qz', '0.63995', '--qw', '0.658381',
                       '--frame-id', 'world', '--child-frame-id', 'rs2_camera_link_calibration_check']
        ),
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='world_to_cam_zed',
            arguments=['--x', '0.100575', '--y', '-0.9304', '--z', '2.31042',
                       '--qx', '0.180663', '--qy', '0.516604', '--qz', '0.119341', '--qw', '0.828395',
                       '--frame-id', 'world', '--child-frame-id', 'zed_camera_link_calibration_check']
        ),
    ])