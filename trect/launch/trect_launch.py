import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():

    ld = LaunchDescription()

    config = os.path.join(
        get_package_share_directory('trect'),
        'config',
        'sim_params.yaml'
        )

    turtlseim_node = Node(
            package='turtlesim',
            namespace='',
            executable='turtlesim_node',
            name='sim',
        )

    trect_node = Node(
            package='trect',
            executable='turtle_rect',
            name='turtle_rect',
            parameters = [config],
            output = 'screen'
        )

    ld.add_action(turtlseim_node)
    ld.add_action(trect_node)

    return ld