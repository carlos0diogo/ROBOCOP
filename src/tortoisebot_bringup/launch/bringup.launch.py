#!/usr/bin/env python3
"""
TortoiseBot Bringup Launch File
Launch principal para inicializar todo o sistema do TortoiseBot
"""

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
    
    # Diretórios dos novos pacotes
    try:
        camera_share = get_package_share_directory('tortoisebot_camera')
        status_share = get_package_share_directory('tortoisebot_status')
    except:
        camera_share = None
        status_share = None
    
    # Caminhos dos arquivos
    default_model_path = os.path.join(pkg_share, 'urdf', 'tortoisebot.xacro')
    default_rviz_config_path = os.path.join(pkg_share, 'rviz', 'tortoisebot_display.rviz')
    
    # Configurações lançáveis
    use_sim_time = LaunchConfiguration('use_sim_time')
    model = LaunchConfiguration('model')
    rvizconfig = LaunchConfiguration('rvizconfig')
    enable_lidar = LaunchConfiguration('enable_lidar')
    enable_camera = LaunchConfiguration('enable_camera')
    enable_status_monitoring = LaunchConfiguration('enable_status_monitoring')
    enable_rviz = LaunchConfiguration('enable_rviz')
    
    # Nós principais
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
    
    # === FIRMWARE NODES ===
    # Controlador diferencial (apenas para robô real)
    differential_drive_node = Node(
        package='tortoisebot_firmware',
        executable='differential_drive',
        name='differential_drive_controller',
        namespace='tortoisebot',
        condition=IfCondition(PythonExpression(['not ', use_sim_time])),
        remappings=[
            ('cmd_vel', '/cmd_vel'),
            ('odom', '/odom'),
        ],
        output='screen'
    )
    
    # Sensor ultrassônico (apenas para robô real)
    ultrasonic_sensor_node = Node(
        package='tortoisebot_firmware',
        executable='ultrasonic_sensor',
        name='ultrasonic_sensor',
        namespace='tortoisebot',
        condition=IfCondition(PythonExpression(['not ', use_sim_time])),
        remappings=[
            ('ultrasonic', '/scan'),
        ],
        output='screen'
    )
    
    # === SENSOR NODES ===
    # LiDAR YDLiDAR
    lidar_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            get_package_share_directory('ydlidar_ros2_driver'),
            '/launch/ydlidar_launch.py'
        ]),
        condition=IfCondition(PythonExpression([enable_lidar, ' and not ', use_sim_time])),
        launch_arguments={
            'port': '/dev/ttyUSB0',
            'frame_id': 'lidar_link',
            'ignore_array': '',
            'baudrate': '115200',
            'lidar_type': '1',
            'device_type': '0',
            'sample_rate': '3',
            'abnormal_check_count': '4'
        }.items()
    )
    
    # Camera
    camera_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            camera_share if camera_share else pkg_share,
            '/launch/camera.launch.py'
        ]),
        condition=IfCondition(PythonExpression([enable_camera, ' and not ', use_sim_time])),
        launch_arguments={
            'camera_index': '0',
            'frame_id': 'camera_link',
            'image_width': '640',
            'image_height': '480',
            'fps': '30'
        }.items()
    ) if camera_share else None
    
    # === STATUS MONITORING ===
    # System monitoring
    status_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            status_share if status_share else pkg_share,
            '/launch/status.launch.py'
        ]),
        condition=IfCondition(enable_status_monitoring),
        launch_arguments={
            'enable_battery_monitor': 'true',
            'enable_system_monitor': 'true',
            'battery_update_rate': '1.0',
            'system_update_rate': '0.5'
        }.items()
    ) if status_share else None
    
    # === VISUALIZATION ===
    differential_drive_node = Node(
        package='tortoisebot_firmware',
        executable='differential_drive',
        name='differential_drive_controller',
        condition=IfCondition(PythonExpression(['not ', use_sim_time])),
        output='screen'
    )
    
    # Sensor ultrassônico (apenas para robô real)
    ultrasonic_sensor_node = Node(
        package='tortoisebot_firmware',
        executable='ultrasonic_sensor',
        name='ultrasonic_sensor',
        condition=IfCondition(PythonExpression(['not ', use_sim_time])),
        output='screen'
    )
    
    # RViz
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rvizconfig],
        parameters=[{'use_sim_time': use_sim_time}],
        condition=IfCondition(enable_rviz)
    )
    
    # === SIMULATION ===
    # Gazebo (apenas para simulação)
    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            get_package_share_directory('gazebo_ros'),
            '/launch/gazebo.launch.py'
        ]),
        condition=IfCondition(use_sim_time),
        launch_arguments={'verbose': 'false'}.items()
    )
    
    # Spawn do robô no Gazebo
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

    # === LAUNCH DESCRIPTION ===
    launch_nodes = [
        # Argumentos do launch
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
            name='enable_lidar',
            default_value='True',
            description='Habilitar LiDAR'
        ),
        DeclareLaunchArgument(
            name='enable_camera',
            default_value='True',
            description='Habilitar câmera'
        ),
        DeclareLaunchArgument(
            name='enable_status_monitoring',
            default_value='True',
            description='Habilitar monitoramento de status'
        ),
        DeclareLaunchArgument(
            name='enable_rviz',
            default_value='True',
            description='Habilitar RViz'
        ),
        
        robot_state_publisher_node,
        joint_state_publisher_node,
        
        # Firmware nodes (robô real)
        differential_drive_node,
        ultrasonic_sensor_node,
        
        # Visualization
        rviz_node,
        
        # Simulation nodes
        gazebo_launch,
        spawn_entity_node,
    ]
    
    # Adicionar launches condicionais se os pacotes existirem
    if camera_share and camera_launch:
        launch_nodes.append(camera_launch)
    
    if status_share and status_launch:
        launch_nodes.append(status_launch)
    
    # Adicionar lidar launch
    launch_nodes.append(lidar_launch)
    
    return LaunchDescription(launch_nodes)
        
        # Nós sempre executados
        robot_state_publisher_node,
        joint_state_publisher_node,
        rviz_node,
        
        # Nós condicionais
        differential_drive_node,
        ultrasonic_sensor_node,
        gazebo_launch,
        spawn_entity_node,
    ])
