#!/usr/bin/env python3
# Author: Alex Mitrevski, Ayush Salunke

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    left_camera_name = LaunchConfiguration('left_camera_name', default='left_camera')
    left_camera_serial_number = LaunchConfiguration('left_camera_serial_number', default='_108322074116')
    right_camera_name = LaunchConfiguration('right_camera_name', default='right_camera')
    right_camera_serial_number = LaunchConfiguration('right_camera_serial_number', default='_049322070507')
    reset_cameras_on_startup = LaunchConfiguration('reset_cameras_on_startup', default=False)
    enable_depth = LaunchConfiguration('enable_depth', default=False)
    camera_colour_stream_profile = LaunchConfiguration('camera_colour_stream_profile', default='640,480,15')
    depth_stream_profile = LaunchConfiguration('depth_stream_profile', default='640,480,15')

    left_realsense_camera_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution([FindPackageShare('realsense2_camera'), 'launch', 'rs_launch.py'])),
        launch_arguments={
            'camera_name': left_camera_name,
            'serial_no': left_camera_serial_number,
            'initial_reset': reset_cameras_on_startup,
            'rgb_camera.color_profile': camera_colour_stream_profile,
            'enable_depth': enable_depth,
            'depth_module.depth_profile': depth_stream_profile
        }.items(),
    )

    right_realsense_camera_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution([FindPackageShare('realsense2_camera'), 'launch', 'rs_launch.py'])),
        launch_arguments={
            'camera_name': right_camera_name,
            'serial_no': right_camera_serial_number,
            'initial_reset': reset_cameras_on_startup,
            'rgb_camera.color_profile': camera_colour_stream_profile,
            'enable_depth': enable_depth,
            'depth_module.depth_profile': depth_stream_profile
        }.items(),
    )

    return LaunchDescription([
        left_realsense_camera_launch,
        right_realsense_camera_launch
    ])
