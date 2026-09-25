"""Lanza TODO el sistema del SCARA con un solo comando.

    ros2 launch scara_bringup scara.launch.py use_fake:=true      # SIN hardware (simulador)
    ros2 launch scara_bringup scara.launch.py use_fake:=false     # ESP32 real por USB serial
    ros2 launch scara_bringup scara.launch.py use_fake:=false serial_port:=/dev/ttyACM0
    ros2 launch scara_bringup scara.launch.py rviz:=false require_homed:=false   # pruebas
    ros2 launch scara_bringup scara.launch.py config:=/ruta/otro.yaml            # otro YAML

use_fake:=false → micro_ros_agent en modo SERIAL:  serial --dev <serial_port> -b <baudrate>
                  (transporte USB, NO UDP)
use_fake:=true  → fake_esp32: la lógica real del firmware + modelo de la planta.

Siempre: scara_bridge · scara_homing · robot_state_publisher (URDF) · RViz2 (config guardada
en scara_bringup/rviz/scara.rviz). Parámetros: scara_bringup/config/scara.yaml.
El agente vive en ~/uros_ws; si ros2_ws se compiló con ese workspace cargado, basta con
"source ros2_ws/install/setup.bash".
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    bringup = FindPackageShare('scara_bringup')
    config = LaunchConfiguration('config')
    desc = FindPackageShare('scara_description')
    urdf = ParameterValue(
        Command(['xacro ', PathJoinSubstitution([desc, 'urdf', 'scara.urdf.xacro'])]),
        value_type=str)

    use_fake = LaunchConfiguration('use_fake')
    serial_port = LaunchConfiguration('serial_port')
    baudrate = LaunchConfiguration('baudrate')

    return LaunchDescription([
        DeclareLaunchArgument('config',
                              default_value=PathJoinSubstitution([bringup, 'config', 'scara.yaml']),
                              description='YAML de parámetros (por defecto el del paquete)'),
        DeclareLaunchArgument('use_fake', default_value='false',
                              description='true = simulador (sin hardware); '
                                          'false = ESP32 real por USB serial'),
        DeclareLaunchArgument('serial_port', default_value='/dev/ttyUSB0',
                              description='puerto del ESP32 (CH9102X): /dev/ttyUSB0 o /dev/ttyACM0'),
        DeclareLaunchArgument('baudrate', default_value='115200',
                              description='igual a UROS_BAUDRATE del firmware'),
        DeclareLaunchArgument('rviz', default_value='true'),
        DeclareLaunchArgument('require_homed', default_value='true',
                              description='rechazar metas sin homing'),

        # --- ESP32 real: agente micro-ROS por SERIAL (USB) ---
        LogInfo(msg=['[scara] modo REAL: micro_ros_agent serial --dev ', serial_port,
                     ' -b ', baudrate], condition=UnlessCondition(use_fake)),
        Node(package='micro_ros_agent', executable='micro_ros_agent', name='micro_ros_agent',
             arguments=['serial', '--dev', serial_port, '-b', baudrate],
             output='screen', condition=UnlessCondition(use_fake)),

        # --- Simulador ---
        LogInfo(msg='[scara] modo SIMULADO: fake_esp32 (lógica real del firmware + planta)',
                condition=IfCondition(use_fake)),
        Node(package='scara_tools', executable='fake_esp32', name='scara_esp32_fake',
             parameters=[config], output='screen', condition=IfCondition(use_fake)),

        # --- Siempre ---
        Node(package='scara_bridge', executable='bridge_node', name='scara_bridge',
             parameters=[config, {'require_homed': ParameterValue(
                 LaunchConfiguration('require_homed'), value_type=bool)}],
             output='screen'),
        Node(package='scara_homing', executable='homing_node', name='scara_homing',
             parameters=[config], output='screen'),
        Node(package='robot_state_publisher', executable='robot_state_publisher',
             parameters=[{'robot_description': urdf}]),
        Node(package='rviz2', executable='rviz2', name='rviz2',
             arguments=['-d', PathJoinSubstitution([bringup, 'rviz', 'scara.rviz'])],
             condition=IfCondition(LaunchConfiguration('rviz'))),
    ])
