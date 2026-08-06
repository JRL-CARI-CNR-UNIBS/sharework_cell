import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    pkg_sharework = get_package_share_directory('sharework_cembre_description')

    # Percorsi dei launcher singoli
    rs_single_launch_path = os.path.join(pkg_sharework, 'launch', 'realsense_single.launch.py')
    zed_single_launch_path = os.path.join(pkg_sharework, 'launch', 'zed_single.launch.py')

    # --- Selettori di Attivazione ---
    enable_rs1_arg = LaunchConfiguration('enable_rs1')
    enable_rs2_arg = LaunchConfiguration('enable_rs2')
    enable_wrist_arg = LaunchConfiguration('enable_wrist')
    enable_zed_arg = LaunchConfiguration('enable_zed')

    # --- Selettori MOCK ---
    global_mock_arg = LaunchConfiguration('mock')

    mock_rs1_arg = LaunchConfiguration('mock_rs1')
    mock_rs2_arg = LaunchConfiguration('mock_rs2')
    mock_wrist_arg = LaunchConfiguration('mock_wrist')
    mock_zed_arg = LaunchConfiguration('mock_zed')

    # 1. RealSense RS1
    rs1_cam = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(rs_single_launch_path),
        condition=IfCondition(enable_rs1_arg),
        launch_arguments={
            'params_file': os.path.join(pkg_sharework, 'config', 'rs1_params.yaml'),
            'param_name_suffix': '1',
            'mock': mock_rs1_arg
        }.items()
    )

    # 2. RealSense RS2
    rs2_cam = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(rs_single_launch_path),
        condition=IfCondition(enable_rs2_arg),
        launch_arguments={
            'params_file': os.path.join(pkg_sharework, 'config', 'rs2_params.yaml'),
            'param_name_suffix': '2',
            'mock': mock_rs2_arg
        }.items()
    )

    # 3. RealSense Wrist
    wrist_cam = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(rs_single_launch_path),
        condition=IfCondition(enable_wrist_arg),
        launch_arguments={
            'params_file': os.path.join(pkg_sharework, 'config', 'wrist_realsense_params.yaml'),
            'param_name_suffix': '3',
            'mock': mock_wrist_arg
        }.items()
    )

    # 4. Telecamera ZED
    zed_cam = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(zed_single_launch_path),
        condition=IfCondition(enable_zed_arg),
        launch_arguments={
            'params_file': os.path.join(pkg_sharework, 'config', 'zed_params.yaml'),
            'calibration_params_file': os.path.join(pkg_sharework, 'config', 'zed_calibration.yaml'),
            'mock': mock_zed_arg
        }.items()
    )

    return LaunchDescription([
        # --- Argomenti di Attivazione (Enable/Disable) ---
        DeclareLaunchArgument('enable_rs1', default_value='true', description='Abilita RealSense RS1'),
        DeclareLaunchArgument('enable_rs2', default_value='true', description='Abilita RealSense RS2'),
        DeclareLaunchArgument('enable_wrist', default_value='true', description='Abilita RealSense Wrist'),
        DeclareLaunchArgument('enable_zed', default_value='true', description='Abilita Telecamera ZED'),

        # --- Argomenti di Gestione MOCK ---
        DeclareLaunchArgument('mock', default_value='false', description='Modalità MOCK globale per tutte le telecamere'),
        DeclareLaunchArgument('mock_rs1', default_value=global_mock_arg, description='Mock specifico per RS1'),
        DeclareLaunchArgument('mock_rs2', default_value=global_mock_arg, description='Mock specifico per RS2'),
        DeclareLaunchArgument('mock_wrist', default_value=global_mock_arg, description='Mock specifico per Wrist'),
        DeclareLaunchArgument('mock_zed', default_value=global_mock_arg, description='Mock specifico per ZED'),

        # Inclusione delle istanze
        rs1_cam,
        rs2_cam,
        wrist_cam,
        zed_cam,
    ])
