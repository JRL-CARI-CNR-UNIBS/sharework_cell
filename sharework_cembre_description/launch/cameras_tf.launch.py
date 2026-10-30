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
            arguments=['--x', '0.0937849', '--y', '-0.929844', '--z', '2.30862',
                       '--qx', '0.185429', '--qy', '0.512749', '--qz', '0.120719', '--qw', '0.829537',
                       '--frame-id', 'world', '--child-frame-id', 'zed_camera_link_calibration_check']
        ),

        # CALIBRATION WITH VERTICAL MOUNTING OF THE CAMERA ON THE WRIST
        # Node(
        #     package='tf2_ros',
        #     executable='static_transform_publisher',
        #     name='camera_link_broadcaster',
        #     arguments=[
        #         '-0.0248083', '-0.0607596', '0.0408365',
        #         '0.474031', '-0.497434', '0.502874', '0.524378',
        #         'ur10e_tool0', 'wrist_link'
        #     ]
        # )

        # CALIBRATION WITH INCLINED MOUNTING OF THE CAMERA ON THE WRIST
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='camera_link_broadcaster',
            arguments=[
                "--frame-id",
                "ur10e_tool0",
                "--child-frame-id",
                "wrist_link",
                "--x",
                "0.00648373",
                "--y",
                "-0.0742079",
                "--z",
                "0.0741869",
                "--qx",
                "0.271469",
                "--qy",
                "-0.27528",
                "--qz",
                "0.644284",
                "--qw",
                "0.659866",
                # "--roll",
                # "0.00506427",
                # "--pitch",
                # "-0.793913",
                # "--yaw",
                # "1.54478",
            ]
        ),


        # VECCHIA CALIBRAZIONE
        # Node(
        #     package='tf2_ros',
        #     executable='static_transform_publisher',
        #     name='camera_link_broadcaster',
        #     arguments=[
        #         "--frame-id",
        #                         "ur10e_tool0",
        #                         "--child-frame-id",
        #                         "wrist_link",
        #                         "--x",
        #                         "0.0174865",
        #                         "--y",
        #                         "-0.0569733",
        #                         "--z",
        #                         "0.0911349",
        #                         "--qx",
        #                         "0.2699",
        #                         "--qy",
        #                         "-0.276147",
        #                         "--qz",
        #                         "0.64937",
        #                         "--qw",
        #                         "0.655145",
        #     ]
        # ),
    ])