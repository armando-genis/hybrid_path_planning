import launch
from launch.substitutions import Command, LaunchConfiguration
import launch_ros
import os
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():

    paramsConfig = os.path.join(get_package_share_directory('dubins_hybrid_star'),'config','params.yaml')


    publisher_node_planner = launch_ros.actions.Node(
        package='dubins_hybrid_star',
        executable='dubins_path_planning_node',
        name='dubins_path_planning_node',
        output='screen',
        parameters=[paramsConfig], 
        emulate_tty=True

    )
    
    return launch.LaunchDescription([
        publisher_node_planner
    ])