import launch
from launch.substitutions import Command, LaunchConfiguration
import launch_ros
import os
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():

    paramsConfig = os.path.join(get_package_share_directory('reeds_shepp_hybrid_astar'),'config','params.yaml')


    publisher_node_planner = launch_ros.actions.Node(
        package='reeds_shepp_hybrid_astar',
        executable='reeds_shepp_path_planning_node',
        name='reeds_shepp_path_planning_node',
        output='screen',
        parameters=[paramsConfig], 
        emulate_tty=True

    )
    
    return launch.LaunchDescription([
        publisher_node_planner
    ])