#!/usr/bin/env python3
# Adapted from a UFACTORY xarm launch
#
# Author: Alex Mitrevski, Ayush Salunke

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    left_arm_ip = LaunchConfiguration('left_arm_ip', default="192.168.1.204")
    right_arm_ip = LaunchConfiguration('right_arm_ip', default="192.168.1.209")
    report_type = LaunchConfiguration('report_type', default='dev')
    
    left_arm_prefix = LaunchConfiguration('left_arm_prefix', default='left_arm_')
    right_arm_prefix = LaunchConfiguration('right_arm_prefix', default='right_arm_')
    hw_ns = LaunchConfiguration('hw_ns', default='xarm')
    limited = LaunchConfiguration('limited', default=False)
    effort_control = LaunchConfiguration('effort_control', default=False)
    velocity_control = LaunchConfiguration('velocity_control', default=False)

    left_arm_add_gripper = LaunchConfiguration('left_arm_add_gripper', default=True)
    left_arm_add_vacuum_gripper = LaunchConfiguration('left_arm_add_vacuum_gripper', default=False)
    right_arm_add_gripper = LaunchConfiguration('right_arm_add_gripper', default=True)
    right_arm_add_vacuum_gripper = LaunchConfiguration('right_arm_add_vacuum_gripper', default=True)

    add_realsense_d435i = LaunchConfiguration('add_realsense_d435i', default=False)

    add_other_geometry = LaunchConfiguration('add_other_geometry', default=False)
    geometry_type = LaunchConfiguration('geometry_type', default='box')
    geometry_mass = LaunchConfiguration('geometry_mass', default=0.1)
    geometry_height = LaunchConfiguration('geometry_height', default=0.1)
    geometry_radius = LaunchConfiguration('geometry_radius', default=0.1)
    geometry_length = LaunchConfiguration('geometry_length', default=0.1)
    geometry_width = LaunchConfiguration('geometry_width', default=0.1)
    geometry_mesh_filename = LaunchConfiguration('geometry_mesh_filename', default='')
    geometry_mesh_origin_xyz = LaunchConfiguration('geometry_mesh_origin_xyz', default='"0 0 0"')
    geometry_mesh_origin_rpy = LaunchConfiguration('geometry_mesh_origin_rpy', default='"0 0 0"')
    geometry_mesh_tcp_xyz = LaunchConfiguration('geometry_mesh_tcp_xyz', default='"0 0 0"')
    geometry_mesh_tcp_rpy = LaunchConfiguration('geometry_mesh_tcp_rpy', default='"0 0 0"')
    
    left_arm_control_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution([FindPackageShare('xarm_controller'), 'launch', '_robot_ros2_control.launch.py'])),
        launch_arguments={
            'robot_ip': left_arm_ip,
            'report_type': report_type,
            'prefix': left_arm_prefix,
            'hw_ns': hw_ns,
            'limited': limited,
            'effort_control': effort_control,
            'velocity_control': velocity_control,
            'add_gripper': left_arm_add_gripper,
            'add_vacuum_gripper': left_arm_add_vacuum_gripper,
            'dof': '6',
            'robot_type': 'xarm',
            'add_realsense_d435i': add_realsense_d435i,
            'add_other_geometry': add_other_geometry,
            'geometry_type': geometry_type,
            'geometry_mass': geometry_mass,
            'geometry_height': geometry_height,
            'geometry_radius': geometry_radius,
            'geometry_length': geometry_length,
            'geometry_width': geometry_width,
            'geometry_mesh_filename': geometry_mesh_filename,
            'geometry_mesh_origin_xyz': geometry_mesh_origin_xyz,
            'geometry_mesh_origin_rpy': geometry_mesh_origin_rpy,
            'geometry_mesh_tcp_xyz': geometry_mesh_tcp_xyz,
            'geometry_mesh_tcp_rpy': geometry_mesh_tcp_rpy,
        }.items(),
    )

    right_arm_control_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution([FindPackageShare('xarm_controller'), 'launch', '_robot_ros2_control.launch.py'])),
        launch_arguments={
            'robot_ip': right_arm_ip,
            'report_type': report_type,
            'prefix': right_arm_prefix,
            'hw_ns': hw_ns,
            'limited': limited,
            'effort_control': effort_control,
            'velocity_control': velocity_control,
            'add_gripper': right_arm_add_gripper,
            'add_vacuum_gripper': right_arm_add_vacuum_gripper,
            'dof': '6',
            'robot_type': 'xarm',
            'add_realsense_d435i': add_realsense_d435i,
            'add_other_geometry': add_other_geometry,
            'geometry_type': geometry_type,
            'geometry_mass': geometry_mass,
            'geometry_height': geometry_height,
            'geometry_radius': geometry_radius,
            'geometry_length': geometry_length,
            'geometry_width': geometry_width,
            'geometry_mesh_filename': geometry_mesh_filename,
            'geometry_mesh_origin_xyz': geometry_mesh_origin_xyz,
            'geometry_mesh_origin_rpy': geometry_mesh_origin_rpy,
            'geometry_mesh_tcp_xyz': geometry_mesh_tcp_xyz,
            'geometry_mesh_tcp_rpy': geometry_mesh_tcp_rpy,
        }.items(),
    )

    return LaunchDescription([
        left_arm_control_launch,
        right_arm_control_launch
    ])
