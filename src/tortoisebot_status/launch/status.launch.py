#!/usr/bin/env python3
"""
Launch file para sistema de monitoramento de status do TortoiseBot
"""

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
import os
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    """Gera a descrição do launch."""
    
    # Diretório do pacote
    pkg_tortoisebot_status = get_package_share_directory('tortoisebot_status')
    
    # Arquivo de configuração padrão
    default_config_file = os.path.join(
        pkg_tortoisebot_status,
        'config',
        'status_config.yaml'
    )
    
    # Argumentos do launch
    config_file_arg = DeclareLaunchArgument(
        'config_file',
        default_value=default_config_file,
        description='Caminho para o arquivo de configuração do status'
    )
    
    enable_battery_monitor_arg = DeclareLaunchArgument(
        'enable_battery_monitor',
        default_value='true',
        description='Habilitar monitoramento de bateria'
    )
    
    enable_system_monitor_arg = DeclareLaunchArgument(
        'enable_system_monitor',
        default_value='true',
        description='Habilitar monitoramento do sistema'
    )
    
    battery_update_rate_arg = DeclareLaunchArgument(
        'battery_update_rate',
        default_value='1.0',
        description='Taxa de atualização do monitor de bateria (Hz)'
    )
    
    system_update_rate_arg = DeclareLaunchArgument(
        'system_update_rate',
        default_value='0.5',
        description='Taxa de atualização do monitor de sistema (Hz)'
    )
    
    # Nó de monitoramento de bateria
    battery_monitor_node = Node(
        package='tortoisebot_status',
        executable='battery_monitor',
        name='battery_monitor',
        namespace='tortoisebot',
        parameters=[
            LaunchConfiguration('config_file'),
            {
                'update_rate': LaunchConfiguration('battery_update_rate'),
                'enable_gpio_monitoring': True,
                'enable_system_monitoring': True,
            }
        ],
        remappings=[
            ('battery_state', '/tortoisebot/status/battery_state'),
            ('battery_voltage', '/tortoisebot/status/battery_voltage'),
            ('battery_percentage', '/tortoisebot/status/battery_percentage'),
            ('low_battery_alert', '/tortoisebot/status/low_battery_alert'),
            ('critical_battery_alert', '/tortoisebot/status/critical_battery_alert'),
        ],
        output='screen',
        condition=LaunchConfiguration('enable_battery_monitor')
    )
    
    # Nó de monitoramento do sistema
    system_monitor_node = Node(
        package='tortoisebot_status',
        executable='system_monitor',
        name='system_monitor',
        namespace='tortoisebot',
        parameters=[
            LaunchConfiguration('config_file'),
            {
                'update_rate': LaunchConfiguration('system_update_rate'),
                'enable_detailed_logging': False,
            }
        ],
        remappings=[
            ('diagnostics', '/tortoisebot/status/diagnostics'),
            ('cpu_temperature', '/tortoisebot/status/cpu_temperature'),
            ('cpu_usage', '/tortoisebot/status/cpu_usage'),
            ('memory_usage', '/tortoisebot/status/memory_usage'),
            ('disk_usage', '/tortoisebot/status/disk_usage'),
            ('system_status', '/tortoisebot/status/system_status'),
        ],
        output='screen',
        condition=LaunchConfiguration('enable_system_monitor')
    )
    
    # Nó aggregador de diagnósticos (opcional)
    diagnostic_aggregator_node = Node(
        package='diagnostic_aggregator',
        executable='aggregator_node',
        name='diagnostic_aggregator',
        namespace='tortoisebot',
        parameters=[{
            'analyzers.tortoisebot.type': 'diagnostic_aggregator/GenericAnalyzer',
            'analyzers.tortoisebot.path': 'TortoiseBot',
            'analyzers.tortoisebot.find_and_remove_prefix': 'tortoisebot',
            'analyzers.tortoisebot.regex': 'tortoisebot/.*',
        }],
        remappings=[
            ('diagnostics', '/tortoisebot/status/diagnostics'),
            ('diagnostics_agg', '/tortoisebot/status/diagnostics_agg'),
        ],
        output='screen'
    )
    
    return LaunchDescription([
        # Argumentos
        config_file_arg,
        enable_battery_monitor_arg,
        enable_system_monitor_arg,
        battery_update_rate_arg,
        system_update_rate_arg,
        
        # Nós
        battery_monitor_node,
        system_monitor_node,
        # diagnostic_aggregator_node,  # Descomente se quiser agregação
    ])
