#!/usr/bin/env python3

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration, Command
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition, UnlessCondition
from launch_ros.actions import Node
from launch_ros.descriptions import ParameterValue

def generate_launch_description():
    
    # Caminhos dos pacotes
    pkg_share = get_package_share_directory('tortoisebot_description')
    default_model_path = os.path.join(pkg_share, 'urdf', 'tortoisebot.xacro')
    default_rviz_config_path = os.path.join(pkg_share, 'rviz', 'tortoisebot_display.rviz')
    
    # Configurações lançáveis
    gui = LaunchConfiguration('gui')
    model = LaunchConfiguration('model')
    rviz_config = LaunchConfiguration('rvizconfig')
    use_sim_time = LaunchConfiguration('use_sim_time')

    return LaunchDescription([
        DeclareLaunchArgument(
            name='gui',
            default_value='True',
            description='Flag para habilitar joint_state_publisher_gui'
        ),
        DeclareLaunchArgument(
            name='model',
            default_value=default_model_path,
            description='Caminho absoluto para o arquivo URDF do robô'
        ),
        DeclareLaunchArgument(
            name='rvizconfig',
            default_value=default_rviz_config_path,
            description='Caminho absoluto para o arquivo de configuração do RViz'
        ),
        DeclareLaunchArgument(
            name='use_sim_time',
            default_value='false',
            description='Usar tempo de simulação'
        ),

        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            parameters=[{
                'robot_description': ParameterValue(
                    Command(['xacro ', model]),
                    value_type=str
                ),
                'use_sim_time': use_sim_time
            }],
            output='screen'
        ),

        Node(
            package='joint_state_publisher_gui',
            executable='joint_state_publisher_gui',
            name='joint_state_publisher_gui',
            condition=IfCondition(gui),
            parameters=[{'use_sim_time': use_sim_time}],
            output='screen'
        ),

        Node(
            package='joint_state_publisher',
            executable='joint_state_publisher',
            name='joint_state_publisher',
            condition=UnlessCondition(gui),
            parameters=[{'use_sim_time': use_sim_time}],
            output='screen'
        ),

        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            arguments=['-d', rviz_config],
            parameters=[{'use_sim_time': use_sim_time}]
        )
    ])
