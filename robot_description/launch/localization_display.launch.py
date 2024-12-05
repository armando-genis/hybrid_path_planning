#!/usr/bin/env python3

import os
import launch
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable
from launch.substitutions import LaunchConfiguration, Command
from launch_ros.actions import Node
import launch_ros

def generate_launch_description():

    urdf_file_name = 'hook.urdf'

    urdf = os.path.join(
        get_package_share_directory('robot_description'),
        'urdf',
        urdf_file_name)

    # Robot State Publisher
    robot_state_publisher_node = launch_ros.actions.Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
                        'robot_description': Command(['xacro ', LaunchConfiguration('model')]),
                    }],
        arguments=[urdf])
    
    # Joint State Publisher
    joint_state_publisher_node = launch_ros.actions.Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
    )
    
    # Static transform: Velodyne -> Base Link (velodyne is the father of base_link)
    static_transform_velodyne_to_base = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_transform_velodyne_to_base',
        # arguments=['-1.2', '0', '-1.55', '0', '-0.1045', '0', '1' ,'velodyne', 'base_link'],
        arguments=['-1.2', '0', '-1.55', '0', '0', '0', '1' , 'velodyne', 'base_link'],
    )

    # Static transform: Base Link -> Vectornav
    static_transform_base_to_vectornav = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_transform_base_to_vectornav',
        arguments=['0.9', '0', '1.3', '0', '0', '0', 'base_link', 'vectornav'],
    )

    # Static transform: Base Link -> Zedd
    static_transform_base_to_zedd = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_transform_base_to_zedd',
        arguments=['2.3', '0', '0', '0', '0', '0', 'base_link', 'zedd'],
    )

    # Static transform: Base Link -> Base Footprint
    static_transform_base_to_base_footprint = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_transform_base_to_base_footprint',
        arguments=['0', '0', '-0.8', '0', '0', '0', 'base_link', 'base_footprint'],
    )

    return launch.LaunchDescription([
        launch.actions.DeclareLaunchArgument(name='model', default_value=urdf,
                                            description='Absolute path to robot urdf file'),

        SetEnvironmentVariable(name='RCUTILS_CONSOLE_OUTPUT_FORMAT', value='{message}'),
        static_transform_velodyne_to_base,
        static_transform_base_to_vectornav,
        static_transform_base_to_zedd,
        static_transform_base_to_base_footprint,  
        joint_state_publisher_node,
        robot_state_publisher_node      
    ])
