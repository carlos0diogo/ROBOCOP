#!/usr/bin/env python3

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration, Command, PythonExpression
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch_ros.actions import Node
from launch_ros.descriptions import ParameterValue
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():
    
    # Diretórios dos pacotes
    pkg_share = get_package_share_directory('tortoisebot_description')
    bringup_share = get_package_share_directory('tortoisebot_bringup')
    
    # Caminhos dos arquivos
    default_model_path = os.path.join(pkg_share, 'urdf', 'tortoisebot.xacro')
    default_rviz_config_path = os.path.join(pkg_share, 'rviz', 'tortoisebot_display.rviz')
    nav2_params_file = os.path.join(bringup_share, 'config', 'nav2_params.yaml')
    map_file = os.path.join(bringup_share, 'maps', 'empty_map.yaml')
    
    # Configurações lançáveis
    use_sim_time = LaunchConfiguration('use_sim_time')
    model = LaunchConfiguration('model')
    rvizconfig = LaunchConfiguration('rvizconfig')
    map_yaml_file = LaunchConfiguration('map')
    exploration = LaunchConfiguration('exploration')
    
    # === NODES BÁSICOS ===
    
    robot_state_publisher_node = Node(
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
    )
    
    joint_state_publisher_node = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        parameters=[{'use_sim_time': use_sim_time}],
        output='screen'
    )
    
    # === HARDWARE NODES (Apenas robô real) ===
    
    differential_drive_node = Node(
        package='tortoisebot_firmware',
        executable='differential_drive',
        name='differential_drive_controller',
        condition=IfCondition(PythonExpression(['not ', use_sim_time])),
        output='screen'
    )
    
    ultrasonic_sensor_node = Node(
        package='tortoisebot_firmware',
        executable='ultrasonic_sensor',
        name='ultrasonic_sensor',
        condition=IfCondition(PythonExpression(['not ', use_sim_time])),
        output='screen'
    )
    
    ydlidar_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            get_package_share_directory('ydlidar_ros2_driver'),
            '/launch/ydlidar_launch.py'
        ]),
        condition=IfCondition(PythonExpression(['not ', use_sim_time]))
    )
    
    # === GAZEBO (Apenas simulação) ===
    
    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            get_package_share_directory('gazebo_ros'),
            '/launch/gazebo.launch.py'
        ]),
        condition=IfCondition(use_sim_time),
        launch_arguments={'verbose': 'false'}.items()
    )
    
    spawn_entity_node = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=[
            '-topic', 'robot_description',
            '-entity', 'tortoisebot',
            '-x', '0',
            '-y', '0', 
            '-z', '0.1'
        ],
        condition=IfCondition(use_sim_time),
        output='screen'
    )
    
    # === SLAM / LOCALIZATION ===
    
    # Cartographer para SLAM (modo exploração)
    cartographer_node = Node(
        package='cartographer_ros',
        executable='cartographer_node',
        name='cartographer_node',
        output='screen',
        condition=IfCondition(exploration),
        parameters=[{'use_sim_time': use_sim_time}],
        arguments=[
            '-configuration_directory', bringup_share + '/config',
            '-configuration_basename', 'cartographer.lua'
        ]
    )
    
    cartographer_occupancy_grid_node = Node(
        package='cartographer_ros',
        executable='cartographer_occupancy_grid_node',
        name='cartographer_occupancy_grid_node',
        output='screen',
        condition=IfCondition(exploration),
        parameters=[{'use_sim_time': use_sim_time}]
    )
    
    # Map server para localização (não exploração)
    map_server_node = Node(
        package='nav2_map_server',
        executable='map_server',
        name='map_server',
        output='screen',
        condition=IfCondition(PythonExpression(['not ', exploration])),
        parameters=[{
            'use_sim_time': use_sim_time,
            'yaml_filename': map_yaml_file
        }]
    )
    
    # === NAVIGATION ===
    
    # Nav2 bringup
    nav2_bringup_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            get_package_share_directory('nav2_bringup'),
            '/launch/navigation_launch.py'
        ]),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'params_file': nav2_params_file
        }.items()
    )
    
    # Lifecycle manager para mapa
    lifecycle_manager_node = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_localization',
        output='screen',
        condition=IfCondition(PythonExpression(['not ', exploration])),
        parameters=[{
            'use_sim_time': use_sim_time,
            'autostart': True,
            'node_names': ['map_server']
        }]
    )
    
    # === VISUALIZATION ===
    
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rvizconfig],
        parameters=[{'use_sim_time': use_sim_time}]
    )
    
    return LaunchDescription([
        # Argumentos
        DeclareLaunchArgument(
            name='use_sim_time',
            default_value='False',
            description='Usar tempo de simulação (True para Gazebo, False para robô real)'
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
            name='map',
            default_value=map_file,
            description='Caminho para o arquivo do mapa'
        ),
        DeclareLaunchArgument(
            name='exploration',
            default_value='True',
            description='Modo exploração (True) ou localização com mapa (False)'
        ),
        
        # Nós básicos (sempre executados)
        robot_state_publisher_node,
        joint_state_publisher_node,
        rviz_node,
        
        # Hardware (apenas robô real)
        differential_drive_node,
        ultrasonic_sensor_node,
        ydlidar_launch,
        
        # Simulação (apenas Gazebo)
        gazebo_launch,
        spawn_entity_node,
        
        # SLAM/Localização
        cartographer_node,
        cartographer_occupancy_grid_node,
        map_server_node,
        lifecycle_manager_node,
        
        # Navegação
        nav2_bringup_launch,
    ])
