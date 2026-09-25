"""Lanza TODO el sistema del SCARA con un solo comando.

    ros2 launch scara_bringup scara.launch.py
    ros2 launch scara_bringup scara.launch.py serial_port:=/dev/ttyACM0 rviz:=false
    ros2 launch scara_bringup scara.launch.py require_homed:=false   # SOLO pruebas

Nodos: micro_ros_agent (serial) · scara_bridge · scara_homing ·
robot_state_publisher (URDF) · RViz2.

El agente vive en otro workspace (~/uros_ws); si ros2_ws se compiló con ese workspace
cargado, basta con "source ros2_ws/install/setup.bash".
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    config = PathJoinSubstitution([FindPackageShare('scara_bringup'), 'config', 'scara.yaml'])
    desc = FindPackageShare('scara_description')
    urdf = ParameterValue(
        Command(['xacro ', PathJoinSubstitution([desc, 'urdf', 'scara.urdf.xacro'])]),
        value_type=str)

    serial_port = LaunchConfiguration('serial_port')
    baudrate = LaunchConfiguration('baudrate')

    return LaunchDescription([
        DeclareLaunchArgument('serial_port', default_value='/dev/ttyUSB0',
                              description='puerto del ESP32 (CH9102X)'),
        DeclareLaunchArgument('baudrate', default_value='115200',
                              description='igual a UROS_BAUDRATE del firmware'),
        DeclareLaunchArgument('agent', default_value='true',
                              description='lanzar micro_ros_agent'),
        DeclareLaunchArgument('rviz', default_value='true'),
        DeclareLaunchArgument('require_homed', default_value='true',
                              description='rechazar metas sin homing'),

        Node(package='micro_ros_agent', executable='micro_ros_agent', name='micro_ros_agent',
             arguments=['serial', '--dev', serial_port, '-b', baudrate],
             output='screen', condition=IfCondition(LaunchConfiguration('agent'))),
        Node(package='scara_bridge', executable='bridge_node', name='scara_bridge',
             parameters=[config, {'require_homed': ParameterValue(
                 LaunchConfiguration('require_homed'), value_type=bool)}],
             output='screen'),
        Node(package='scara_homing', executable='homing_node', name='scara_homing',
             parameters=[config], output='screen'),
        Node(package='robot_state_publisher', executable='robot_state_publisher',
             parameters=[{'robot_description': urdf}]),
        Node(package='rviz2', executable='rviz2', name='rviz2',
             arguments=['-d', PathJoinSubstitution([desc, 'rviz', 'scara.rviz'])],
             condition=IfCondition(LaunchConfiguration('rviz'))),
    ])
