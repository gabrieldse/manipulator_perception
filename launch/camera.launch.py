# Copyright 2018 Lucas Walter
# All rights reserved.
#
# Software License Agreement (BSD License 2.0)
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
#  * Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
#  * Redistributions in binary form must reproduce the above
#    copyright notice, this list of conditions and the following
#    disclaimer in the documentation and/or other materials provided
#    with the distribution.
#  * Neither the name of Lucas Walter nor the names of its
#    contributors may be used to endorse or promote products derived
#    from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.

# import argparse
# import os
# from pathlib import Path  # noqa: E402
# import sys

# # Hack to get relative import of .camera_config file working
# dir_path = os.path.dirname(os.path.realpath(__file__))
# sys.path.append(dir_path)

# from camera_config import CameraConfig #, USB_CAM_DIR  # noqa: E402

# from launch import LaunchDescription  # noqa: E402
# from launch.actions import GroupAction  # noqa: E402
# from launch_ros.actions import Node  # noqa: E402


# CAMERAS = []
# CAMERAS.append(
#     CameraConfig(
#         name='camera1',
#         param_path=Path('/root/dev_ws/src/manipulator_h_vision/', 'config', 'params_1.yaml')
#     )
#     # Add more Camera's here and they will automatically be launched below
# )


# def generate_launch_description():
#     ld = LaunchDescription()

#     parser = argparse.ArgumentParser(description='usb_cam demo')
#     parser.add_argument('-n', '--node-name', dest='node_name', type=str,
#                         help='name for device', default='usb_cam')

#     camera_nodes = [
#         Node(
#             package='usb_cam', executable='usb_cam_node_exe', output='screen',
#             name=camera.name,
#             namespace=camera.namespace,
#             parameters=[camera.param_path],
#             remappings=camera.remappings
#         )
#         for camera in CAMERAS
#     ]

#     camera_group = GroupAction(camera_nodes)

#     ld.add_action(camera_group)
#     return ld

import os
from pathlib import Path

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

# Camera configuration class
class CameraConfig:
    def __init__(self, name, param_path, namespace=""):
        self.name = name
        self.param_path = param_path
        self.namespace = namespace
        self.remappings = []

# Get the path to the configuration file dynamically
param_default_file_path = os.path.join(
    get_package_share_directory('manipulator_h_vision'), 'config', 'params.yaml'
)

node_name = LaunchConfiguration('node_name')
param_path = LaunchConfiguration('param_path')

# List of cameras (You can add more cameras here)
camera = CameraConfig(name=node_name, param_path=param_path)

def generate_launch_description():
    """Generates the launch description for starting USB cameras"""
    
    ld = LaunchDescription()

    # Declare launch argument for the node name
    ld.add_action(DeclareLaunchArgument(
        'node_name', default_value='usb_cam', description='Name for the USB camera node'
    ))
    
    ld.add_action(DeclareLaunchArgument(
        'param_path', default_value=param_default_file_path, description='Path to the camera configuration file'
    ))

    camera_node = Node(
            package='usb_cam',
            executable='usb_cam_node_exe',
            output='screen',
            name=camera.name,
            namespace=camera.namespace,
            parameters=[camera.param_path],
            remappings=camera.remappings
        )
    
    camera_viewer = Node(package='usb_cam', executable='show_image', name='show_image', output='screen')


    ld.add_action(camera_node)
    
    return ld
