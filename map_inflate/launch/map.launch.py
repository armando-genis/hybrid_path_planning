import launch
from launch.substitutions import Command, LaunchConfiguration
import launch_ros
import os
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():

    paramsConfig = os.path.join(get_package_share_directory('map_inflate'),'config','params.yaml')


    publisher_node_planner = launch_ros.actions.Node(
        package='map_inflate',
        executable='map_inflate_node',
        name='map_inflate_node',
        output='screen',
        parameters=[paramsConfig],
        additional_env={'RCUTILS_CONSOLE_OUTPUT_FORMAT': "{message}"}
    )
    
    return launch.LaunchDescription([
        publisher_node_planner
    ])