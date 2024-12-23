# Copyright 2023 Intel Corporation. All Rights Reserved.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

# DESCRIPTION #
# ----------- #
# Use this launch file to launch 2 devices.
# The Parameters available for definition in the command line for each camera are described in rs_launch.configurable_parameters
# For each device, the parameter name was changed to include an index.
# For example: to set camera_name for device1 set parameter camera_name1.
# command line example:
# ros2 launch realsense2_camera rs_dual_camera_launch.py serial_no1:=<serial number of 1st camera> serial_no2:=<serial number of 2nd camera>

import copy
from launch import LaunchDescription
from launch.actions import OpaqueFunction
from launch.substitutions import LaunchConfiguration
import sys
import pathlib
sys.path.append(str(pathlib.Path(__file__).parent.absolute()))
import os
from ament_index_python.packages import get_package_share_directory
sys.path.append(os.path.join(get_package_share_directory('realsense2_camera'), 'launch'))
import rs_launch # type: ignore

local_parameters = [
    {'name': 'camera_name1',                 'default': 'rs1',           'description': 'camera1 unique name'},
    {'name': 'camera_name2',                 'default': 'rs2',           'description': 'camera2 unique name'},
    {'name': 'serial_no1',                   'default': '_025222071790', 'description': 'choose device1 by serial number'},
    {'name': 'serial_no2',                   'default': '_207522077984', 'description': 'choose device2 by serial number'},
    {'name': 'camera_namespace1',            'default': 'rs1',           'description': 'camera1 namespace'},
    {'name': 'camera_namespace2',            'default': 'rs2',           'description': 'camera2 namespace'},
    {'name': 'enable_color1',                'default': 'true',          'description': 'enable color stream'},
    {'name': 'enable_color2',                'default': 'true',          'description': 'enable color stream'},
    {'name': 'enable_depth1',                'default': 'true',          'description': 'enable depth stream'},
    {'name': 'enable_depth2',                'default': 'true',          'description': 'enable depth stream'},
    {'name': 'pointcloud.enable1',           'default': 'true',          'description': 'enable pointcloud'},
    {'name': 'pointcloud.enable2',           'default': 'true',          'description': 'enable pointcloud'},
    {'name': 'spatial_filter.enable1',       'default': 'true',          'description': 'enable_spatial_filter'},
    {'name': 'spatial_filter.enable2',       'default': 'true',          'description': 'enable_spatial_filter'},
    {'name': 'temporal_filter.enable1',      'default': 'true',          'description': 'enable_temporal_filter'},
    {'name': 'temporal_filter.enable2',      'default': 'true',          'description': 'enable_temporal_filter'},

    {'name': 'decimation_filter.enable1',    'default': 'false',          'description': 'enable_decimation_filter'},
    {'name': 'decimation_filter.enable2',    'default': 'false',          'description': 'enable_decimation_filter'},
    {'name': 'hole_filling_filter.enable1',  'default': 'true',          'description': 'enable_hole_filling_filter'},
    {'name': 'hole_filling_filter.enable2',  'default': 'true',          'description': 'enable_hole_filling_filter'},

    {'name': 'rgb_camera.color_profile1',    'default': '1280x720x30',   'description': 'RGB camera profile'},
    {'name': 'rgb_camera.color_profile2',    'default': '1280x720x30',   'description': 'RGB camera profile'},
    {'name': 'depth_module.depth_profile1',  'default': '848x480x30',    'description': 'Depth camera profile'},
    {'name': 'depth_module.depth_profile2',  'default': '848x480x30',    'description': 'Depth camera profile'},
    {'name': 'align_depth.enable1',          'default': 'false',         'description': 'enable depth alignment'},
    {'name': 'align_depth.enable2',          'default': 'false',         'description': 'enable depth alignment'},
    {'name': 'enable_sync1',                 'default': 'false',         'description': 'enable synchronization'},
    {'name': 'enable_sync2',                 'default': 'false',         'description': 'enable synchronization'}                
]

def set_configurable_parameters(local_params):
    return dict([(param['original_name'], LaunchConfiguration(param['name'])) for param in local_params])

def duplicate_params(general_params, posix):
    local_params = copy.deepcopy(general_params)
    for param in local_params:
        param['original_name'] = param['name']
        param['name'] += posix
    return local_params

def generate_launch_description():
    params1 = duplicate_params(rs_launch.configurable_parameters, '1')
    params2 = duplicate_params(rs_launch.configurable_parameters, '2') 
    return LaunchDescription(
        rs_launch.declare_configurable_parameters(local_parameters) +
        rs_launch.declare_configurable_parameters(params1) + 
        rs_launch.declare_configurable_parameters(params2) + 
        [
            OpaqueFunction(function=rs_launch.launch_setup,
                kwargs = {'params': set_configurable_parameters(params1),
                'param_name_suffix': '1'}
            ),
            OpaqueFunction(function=rs_launch.launch_setup,
                kwargs = {'params': set_configurable_parameters(params2),
                'param_name_suffix': '2'}
            )
        ]
    )
