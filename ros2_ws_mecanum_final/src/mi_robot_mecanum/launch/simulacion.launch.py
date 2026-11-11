"""
Lanza la pista en Gazebo, el robot Mecanum, el bridge ROS 2 <-> Gazebo,
el nodo de cinemática y RViz — Examen final Robótica Móvil.

    ros2 launch mi_robot_mecanum simulacion.launch.py carne:=<su_carne>
"""
import os
import sys

import xacro
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction, LogInfo
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from sdf_mecanum import escribir_sdf_mecanum  # noqa: E402  (dado, no modificar)
from mi_mecanum_control.datos_carne import datos, texto  # noqa: E402


def configurar(context):
    pkg = get_package_share_directory('mi_robot_mecanum')
    D = datos(LaunchConfiguration('carne').perform(context))
    xacro_file = os.path.join(pkg, 'urdf', 'mi_robot.urdf.xacro')
    world_file = os.path.join(pkg, 'worlds', 'pista.sdf')
    rviz_file = os.path.join(pkg, 'rviz', 'control.rviz')

    args_xacro = {'rueda_radio': str(D['rueda_radio']),
                  'distancia_ejes': str(D['distancia_ejes']),
                  'separacion_ruedas': str(D['separacion_ruedas'])}
    urdf = xacro.process_file(xacro_file, mappings=args_xacro).toxml()
    sdf_file = escribir_sdf_mecanum(urdf)     # SDF con la fricción de los rodillos

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            get_package_share_directory('ros_gz_sim'), 'launch', 'gz_sim.launch.py')),
        launch_arguments={'gz_args': f'-r {world_file}'}.items(),
    )

    robot_state_publisher = Node(
        package='robot_state_publisher', executable='robot_state_publisher',
        parameters=[{'robot_description': ParameterValue(urdf, value_type=str), 'use_sim_time': True}],
        output='screen',
    )

    spawn = Node(
        package='ros_gz_sim', executable='create',
        arguments=['-name', 'mi_robot', '-file', sdf_file,
                   '-x', '0.0', '-y', '0.0', '-z', str(D['rueda_radio'] + 0.01)],
        output='screen',
    )

    # ]  ROS -> Gazebo     [  Gazebo -> ROS
    bridge = Node(
        package='ros_gz_bridge', executable='parameter_bridge',
        arguments=[
            '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
            '/vel_rueda_del_izq@std_msgs/msg/Float64]gz.msgs.Double',
            '/vel_rueda_del_der@std_msgs/msg/Float64]gz.msgs.Double',
            '/vel_rueda_tra_izq@std_msgs/msg/Float64]gz.msgs.Double',
            '/vel_rueda_tra_der@std_msgs/msg/Float64]gz.msgs.Double',
            '/world/pista/model/mi_robot/joint_state@sensor_msgs/msg/JointState[gz.msgs.Model',
            '/scan@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan',
        ],
        remappings=[('/world/pista/model/mi_robot/joint_state', '/joint_states')],
        parameters=[{'use_sim_time': True}],
        output='screen',
    )

    cinematica = Node(
        package='mi_mecanum_control', executable='cinematica',
        parameters=[{'rueda_radio': float(D['rueda_radio']),
                     'distancia_ejes': float(D['distancia_ejes']),
                     'separacion_ruedas': float(D['separacion_ruedas']),
                     'use_sim_time': True}],
        output='screen',
    )

    rviz_node = Node(
        package='rviz2', executable='rviz2', arguments=['-d', rviz_file],
        parameters=[{'use_sim_time': True}],
        condition=IfCondition(LaunchConfiguration('rviz')),
    )

    return [LogInfo(msg='\n' + texto(D)), gazebo, robot_state_publisher, spawn, bridge,
            cinematica, rviz_node]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('carne', description='Carné completo del estudiante'),
        DeclareLaunchArgument('rviz', default_value='true'),
        OpaqueFunction(function=configurar),
    ])
