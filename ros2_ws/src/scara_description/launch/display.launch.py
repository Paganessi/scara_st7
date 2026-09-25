"""Ver el modelo SIN robot: sliders (joint_state_publisher_gui) + RViz.

    ros2 launch scara_description display.launch.py

Sirve para revisar el URDF y los límites antes de conectar el hardware.
"""
from launch import LaunchDescription
from launch.substitutions import Command, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    share = FindPackageShare('scara_description')
    urdf = ParameterValue(
        Command(['xacro ', PathJoinSubstitution([share, 'urdf', 'scara.urdf.xacro'])]),
        value_type=str)
    return LaunchDescription([
        Node(package='robot_state_publisher', executable='robot_state_publisher',
             parameters=[{'robot_description': urdf}]),
        Node(package='joint_state_publisher_gui', executable='joint_state_publisher_gui'),
        Node(package='rviz2', executable='rviz2',
             arguments=['-d', PathJoinSubstitution([share, 'rviz', 'scara.rviz'])]),
    ])
