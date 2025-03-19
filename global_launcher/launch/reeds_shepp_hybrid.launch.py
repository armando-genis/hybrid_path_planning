import launch
from launch.substitutions import Command, LaunchConfiguration
import launch_ros
import os
from ament_index_python.packages import get_package_share_directory
from launch.actions import TimerAction, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():

    Map_paramsConfig = os.path.join(get_package_share_directory('global_launcher'),'config','map_config.yaml')
    planner_paramsConfig = os.path.join(get_package_share_directory('global_launcher'),'config','reeds_config.yaml')
    rviz_config_dir = os.path.join(get_package_share_directory('global_launcher'), 'rviz', 'planner.rviz')


    map_to_robot = launch_ros.actions.Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_transform_velodyne_to_base',
        arguments=['0', '0', '0', '0', '0', '0', '1' , 'map', 'base_link'],
    )

    publisher_node_grid_map = launch_ros.actions.Node(
        package='grid_map_publisher',
        executable='grid_map_publisher',
        name='grid_map_publisher',
        output='screen',
        parameters=[Map_paramsConfig], 
        additional_env={'RCUTILS_CONSOLE_OUTPUT_FORMAT': "{message}"}
    )

    publisher_node_map_inflate = launch_ros.actions.Node(
        package='map_inflate',
        executable='map_inflate_node',
        name='map_inflate_node',
        output='screen',
        parameters=[Map_paramsConfig],
        additional_env={'RCUTILS_CONSOLE_OUTPUT_FORMAT': "{message}"}
    )

    publisher_node_planner = launch_ros.actions.Node(
        package='reeds_shepp_hybrid_astar',
        executable='reeds_shepp_path_planning_node',
        name='reeds_shepp_path_planning_node',
        output='screen',
        parameters=[planner_paramsConfig],
        additional_env={'RCUTILS_CONSOLE_OUTPUT_FORMAT': "{message}"}
    )

    rviz_node = launch_ros.actions.Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', LaunchConfiguration('rvizconfig')]
    )

    # Static Transform Publisher (map -> odom)
    static_tf_node = launch_ros.actions.Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_transform_publisher',
        arguments=['0', '0', '0', '0', '0', '0', 'map', 'odom'],
        output='screen'
    )

        # ros2 launch robot_description localization_display.launch.py
    robot_description_for_localization = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('robot_description'),
                'launch',
                'localization_display.launch.py'
            ])
        ]),
    )

    
    return launch.LaunchDescription([

        launch.actions.DeclareLaunchArgument(name='rvizconfig', default_value=rviz_config_dir,
                                            description='Absolute path to rviz config file'),
        # static_tf_node,
        robot_description_for_localization,
        publisher_node_grid_map,
        publisher_node_map_inflate,
        map_to_robot,
        TimerAction(
            actions=[
                publisher_node_planner,
                rviz_node
            ],
            period='2.0',  
        ),
    ])