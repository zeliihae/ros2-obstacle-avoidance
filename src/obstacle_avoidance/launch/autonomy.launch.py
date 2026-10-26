"""Simulasyon + perception + decision + control dugumlerini tek komutla baslatir.

    ros2 launch obstacle_avoidance autonomy.launch.py

NOT: Terminal 4'te elle /cmd_vel yayinlamayin; control_node ile cakisir.
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():
    pkg = get_package_share_directory('obstacle_avoidance')
    params = os.path.join(pkg, 'config', 'avoidance_params.yaml')

    simulation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg, 'launch', 'simulation.launch.py')))

    perception = Node(package='obstacle_avoidance', executable='perception_node',
                      name='perception_node', output='screen')
    decision = Node(package='obstacle_avoidance', executable='decision_node',
                    name='decision_node', output='screen', parameters=[params])
    control = Node(package='obstacle_avoidance', executable='control_node',
                   name='control_node', output='screen', parameters=[params])

    # Gazebo'nun acilmasi icin dugumleri 5 sn gecikmeyle baslat.
    return LaunchDescription([
        simulation,
        TimerAction(period=5.0, actions=[perception, decision, control]),
    ])