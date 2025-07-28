#!/usr/bin/env python3
"""
Launch file para o sistema de câmera do TortoiseBot
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
    pkg_tortoisebot_camera = get_package_share_directory('tortoisebot_camera')
    
    # Arquivo de configuração padrão
    default_config_file = os.path.join(
        pkg_tortoisebot_camera,
        'config',
        'camera_config.yaml'
    )
    
    # Argumentos do launch
    camera_config_file_arg = DeclareLaunchArgument(
        'camera_config_file',
        default_value=default_config_file,
        description='Caminho para o arquivo de configuração da câmera'
    )
    
    camera_index_arg = DeclareLaunchArgument(
        'camera_index',
        default_value='0',
        description='Índice da câmera (0, 1, 2...)'
    )
    
    frame_id_arg = DeclareLaunchArgument(
        'frame_id',
        default_value='camera_link',
        description='Frame ID da câmera'
    )
    
    image_width_arg = DeclareLaunchArgument(
        'image_width',
        default_value='640',
        description='Largura da imagem em pixels'
    )
    
    image_height_arg = DeclareLaunchArgument(
        'image_height',
        default_value='480',
        description='Altura da imagem em pixels'
    )
    
    fps_arg = DeclareLaunchArgument(
        'fps',
        default_value='30',
        description='Taxa de quadros por segundo'
    )
    
    auto_exposure_arg = DeclareLaunchArgument(
        'auto_exposure',
        default_value='true',
        description='Exposição automática (true/false)'
    )
    
    # Nó da câmera
    camera_node = Node(
        package='tortoisebot_camera',
        executable='camera_node',
        name='camera_node',
        namespace='tortoisebot',
        parameters=[
            LaunchConfiguration('camera_config_file'),
            {
                'camera_index': LaunchConfiguration('camera_index'),
                'frame_id': LaunchConfiguration('frame_id'),
                'image_width': LaunchConfiguration('image_width'),
                'image_height': LaunchConfiguration('image_height'),
                'fps': LaunchConfiguration('fps'),
                'auto_exposure': LaunchConfiguration('auto_exposure'),
            }
        ],
        remappings=[
            ('image_raw', '/tortoisebot/camera/image_raw'),
            ('camera_info', '/tortoisebot/camera/camera_info'),
        ],
        output='screen'
    )
    
    # Image transport plugins (opcional)
    # Pode ser usado para compressão de imagem
    compressed_publisher = Node(
        package='image_transport',
        executable='republish',
        name='compressed_republisher',
        namespace='tortoisebot',
        arguments=[
            'raw', 'in:=/tortoisebot/camera/image_raw',
            'compressed', 'out:=/tortoisebot/camera/image_raw'
        ],
        output='screen'
    )
    
    return LaunchDescription([
        # Argumentos
        camera_config_file_arg,
        camera_index_arg,
        frame_id_arg,
        image_width_arg,
        image_height_arg,
        fps_arg,
        auto_exposure_arg,
        
        # Nós
        camera_node,
        # compressed_publisher,  # Descomente se quiser compressão
    ])
