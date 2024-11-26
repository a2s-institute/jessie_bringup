#!/usr/bin/env python3
# Software License Agreement (BSD License)
#
# Copyright (c) 2021, UFACTORY, Inc.
# All rights reserved.
#
# Author: Vinman <vinman.wen@ufactory.cc> <vinman.cub@gmail.com>

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    left_camera_name = LaunchConfiguration('left_camera_name', default='left_camera')
    left_camera_serial_number = LaunchConfiguration('left_camera_serial_number', default='108322074116')
    left_camera_usb_port_id = LaunchConfiguration('left_camera_serial_number', default='2-2.2')
    right_camera_name = LaunchConfiguration('right_camera_name', default='right_camera')
    right_camera_serial_number = LaunchConfiguration('right_camera_serial_number', default='049322070507')
    right_camera_usb_port_id = LaunchConfiguration('right_camera_serial_number', default='2-2.4')
    reset_cameras_on_startup = LaunchConfiguration('reset_cameras_on_startup', default=False)
    camera_colour_stream_profile = LaunchConfiguration('camera_colour_stream_profile', default='640x480x15')
    depth_stream_profile = LaunchConfiguration('depth_stream_profile', default='640x480x15')

    left_realsense_camera_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution([FindPackageShare('realsense2_camera'), 'launch', 'rs_launch.py'])),
        launch_arguments={
            'camera_name': left_camera_name,
            'serial_no': str(left_camera_serial_number)
            #'usb_port_id': left_camera_usb_port_id,
            'initial_reset': reset_cameras_on_startup,
            'rgb_camera.profile': camera_colour_stream_profile,
            'depth_module.profile': depth_stream_profile
        }.items(),
    )

    right_realsense_camera_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution([FindPackageShare('realsense2_camera'), 'launch', 'rs_launch.py'])),
        launch_arguments={
            'camera_name': right_camera_name,
            'serial_no': str(right_camera_serial_number)
            #'usb_port_id': right_camera_usb_port_id,
            'initial_reset': reset_cameras_on_startup,
            'rgb_camera.profile': camera_colour_stream_profile,
            'depth_module.profile': depth_stream_profile
        }.items(),
    )

    return LaunchDescription([
        left_realsense_camera_launch,
        right_realsense_camera_launch
    ])
