import os
from pathlib import Path

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, LogInfo
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
    get_package_share_directory('ring_tracker'), 'config', 'params.yaml'
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
    
    ring_tracker_node = Node(
            package='ring_tracker',
            executable='ring_tracking_node',
            output='screen',
        )
    
    camera_viewer = Node(package='usb_cam', executable='show_image', name='show_image', output='screen')

    ld.add_action(camera_node)    
    ld.add_action(
        GroupAction(
            actions=[LogInfo(msg="Waiting for camera node to be ready..."), ring_tracker_node]
            )
        )
    
    return ld
