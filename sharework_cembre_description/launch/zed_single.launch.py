import os
import yaml
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def launch_setup(context, *args, **kwargs):
    pkg_zed_wrapper = get_package_share_directory('zed_wrapper')
    
    params_file_path = LaunchConfiguration('params_file').perform(context)
    calib_file_path = LaunchConfiguration('calibration_params_file').perform(context)
    mock_val = LaunchConfiguration('mock').perform(context).lower() == 'true'

    # Carica i parametri dal file di calibrazione dedicato
    with open(calib_file_path, 'r') as f:
        calib_data = yaml.safe_load(f)

    calib_params = calib_data.get('/**', {}).get('ros__parameters', {}).get('calibration_tf', {})
    parent_frame = calib_params.get('parent_frame', 'world')
    child_frame = calib_params.get('child_frame', 'zed_camera_link_calibration_check')
    trans = calib_params.get('translation', [0.0937849, -0.929844, 2.30862])
    rot = calib_params.get('rotation_quaternion', [0.185429, 0.512749, 0.120719, 0.829537])

    # Carica il file di parametri hardware originale per ricavare nome/modello
    with open(params_file_path, 'r') as f:
        params_data = yaml.safe_load(f)

    ros_params = params_data.get('/**', {}).get('ros__parameters', {})
    camera_name = ros_params.get('general', {}).get('camera_name', 'zed')
    camera_model = ros_params.get('general', {}).get('camera_model', 'zed2')

    # 1. Pubblicazione TF Statica di Calibrazione
    calibration_tf_node = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name=f'{camera_name}_calibration_tf_pub',
        arguments=[
            '--x', str(trans[0]), '--y', str(trans[1]), '--z', str(trans[2]),
            '--qx', str(rot[0]), '--qy', str(rot[1]), '--qz', str(rot[2]), '--qw', str(rot[3]),
            '--frame-id', parent_frame,
            '--child-frame-id', child_frame
        ]
    )

    nodes_to_start = [calibration_tf_node]

    if mock_val:
        # Modo MOCK: Collega il child_frame di calibrazione al camera_link base simulato
        mock_tf_node = Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name=f'{camera_name}_mock_tf_pub',
            arguments=[
                '--x', '0', '--y', '0', '--z', '0',
                '--roll', '0', '--pitch', '0', '--yaw', '0',
                '--frame-id', child_frame,
                '--child-frame-id', f'{camera_name}_camera_link'
            ]
        )
        nodes_to_start.append(mock_tf_node)
    else:
        # Modo REALE: Passa il file di parametri originale intatto a zed_camera.launch.py
        zed_driver = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(pkg_zed_wrapper, 'launch', 'zed_camera.launch.py')
            ),
            launch_arguments={
                'camera_name': camera_name,
                'camera_model': camera_model,
                'ros_params_override_path': params_file_path,
            }.items()
        )
        nodes_to_start.append(zed_driver)

    return nodes_to_start


def generate_launch_description():
    pkg_sharework = get_package_share_directory('sharework_cembre_description')

    return LaunchDescription([
        DeclareLaunchArgument(
            'params_file',
            default_value=os.path.join(pkg_sharework, 'config', 'zed_params.yaml'),
            description='Percorso al file YAML dei parametri della ZED'
        ),
        DeclareLaunchArgument(
            'calibration_params_file',
            default_value=os.path.join(pkg_sharework, 'config', 'zed_calibration.yaml'),
            description='Percorso al file YAML contenente la TF di calibrazione'
        ),
        DeclareLaunchArgument(
            'mock',
            default_value='false',
            description='Se true, disabilita il nodo hardware ZED e attiva le TF simulate'
        ),
        OpaqueFunction(function=launch_setup)
    ])
