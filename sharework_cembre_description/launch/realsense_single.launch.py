import os
import sys
import copy
import yaml
import math

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

# Import rs_launch dal pacchetto realsense2_camera
sys.path.append(os.path.join(get_package_share_directory('realsense2_camera'), 'launch'))
import rs_launch  # type: ignore


def set_configurable_parameters(local_params):
    return dict([(param['original_name'], LaunchConfiguration(param['name'])) for param in local_params])


def duplicate_params(general_params, posix):
    local_params = copy.deepcopy(general_params)
    for param in local_params:
        param['original_name'] = param['name']
        param['name'] += posix
    return local_params


def launch_nodes_with_config(context, *args, **kwargs):
    params_file_path = LaunchConfiguration('params_file').perform(context)
    suffix = LaunchConfiguration('param_name_suffix').perform(context)
    mock_arg = LaunchConfiguration('mock').perform(context).lower()
    is_mock = mock_arg in ['true', '1', 'yes']

    # Carica il file YAML di configurazione
    with open(params_file_path, 'r') as f:
        config = yaml.safe_load(f)

    entities = []

    # Nome della telecamera (default fallback: "camera")
    camera_name = config.get('camera_name', 'camera')
    child_frame = f"{camera_name}_link"

    # =========================================================================
    # 1. TF Statica Principale (Parent -> Camera Link) - Sempre pubblicata
    # =========================================================================
    tf_cfg = config.get('tf', {})
    if tf_cfg:
        translation = [str(v) for v in tf_cfg.get('translation', ['0', '0', '0'])]
        rotation = [str(v) for v in tf_cfg.get('rotation', ['0', '0', '0', '1'])]
        parent_frame = str(tf_cfg.get('parent_frame', 'world'))

        # Argomenti per static_transform_publisher: x y z qx qy qz qw parent_frame child_frame
        tf_args = translation + rotation + [parent_frame, child_frame]

        entities.append(
            Node(
                package='tf2_ros',
                executable='static_transform_publisher',
                name=f'{camera_name}_static_tf_publisher',
                arguments=tf_args
            )
        )

    # =========================================================================
    # 2. Gestione modalità REAL vs MOCK
    # =========================================================================
    if not is_mock:
        # --- MODALITÀ REALE: Avvia la telecamera RealSense ---
        params_duplicated = duplicate_params(rs_launch.configurable_parameters, suffix)

        # Assegna i default letti dallo YAML se presenti
        for p in params_duplicated:
            param_key = p['original_name']
            if param_key in config:
                p['default'] = str(config[param_key])

        entities.extend(rs_launch.declare_configurable_parameters(params_duplicated))

        entities.append(
            OpaqueFunction(
                function=rs_launch.launch_setup,
                kwargs={
                    'params': set_configurable_parameters(params_duplicated),
                    'param_name_suffix': suffix
                }
            )
        )

    else:
        # --- MODALITÀ MOCK: Pubblica solo la TF statica del frame ottico simulato ---
        # RPY: roll = -pi/2, pitch = 0, yaw = -pi/2
        half_pi = str(-math.pi / 2) # "-1.57079632679"

        # Sintassi TF2 con flag RPY per tf2_ros: --x --y --z --roll --pitch --yaw --frame-id --child-frame-id
        # Oppure sintassi posizionale RPY: x y z roll pitch yaw parent child
        mock_optical_tf_args = [
            '0', '0', '0',                      # xyz
            half_pi, '0', half_pi,             # rpy (-pi/2, 0, -pi/2)
            child_frame,                       # parent: ${camera_name}_link
            f"{camera_name}_color_optical_frame" # child: ${camera_name}_color_optical_frame
        ]

        entities.append(
            Node(
                package='tf2_ros',
                executable='static_transform_publisher',
                name=f'{camera_name}_mock_optical_tf_publisher',
                arguments=mock_optical_tf_args
            )
        )

    return entities


def generate_launch_description():
    default_yaml_path = os.path.join(
        get_package_share_directory('sharework_cembre_description'),
        'config',
        'wrist_realsense_params.yaml'
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'params_file',
            default_value=default_yaml_path,
            description='Percorso assoluto al file YAML dei parametri per la telecamera'
        ),
        DeclareLaunchArgument(
            'param_name_suffix',
            default_value='',
            description='Soffisso opzionale per i parametri della telecamera'
        ),
        DeclareLaunchArgument(
            'mock',
            default_value='false',
            description='Se true, non avvia il driver della fotocamera ma pubblica una TF statica mock'
        ),
        OpaqueFunction(function=launch_nodes_with_config)
    ])
