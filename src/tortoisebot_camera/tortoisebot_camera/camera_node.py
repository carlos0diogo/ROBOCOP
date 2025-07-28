#!/usr/bin/env python3
"""
TortoiseBot Camera Node
Controla a webcam Lenovo 300 e publica imagens via ROS2
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge
import cv2
import numpy as np
from camera_info_manager import CameraInfoManager


class CameraNode(Node):
    """Nó ROS2 para controle da câmera Lenovo 300."""
    
    def __init__(self):
        super().__init__('camera_node')
        
        # Parâmetros
        self.declare_parameter('camera_index', 0)
        self.declare_parameter('frame_id', 'camera_link')
        self.declare_parameter('image_width', 640)
        self.declare_parameter('image_height', 480)
        self.declare_parameter('fps', 30)
        self.declare_parameter('camera_info_file', '')
        self.declare_parameter('auto_exposure', True)
        self.declare_parameter('brightness', 50)
        self.declare_parameter('contrast', 50)
        
        # Obter parâmetros
        self.camera_index = self.get_parameter('camera_index').value
        self.frame_id = self.get_parameter('frame_id').value
        self.image_width = self.get_parameter('image_width').value
        self.image_height = self.get_parameter('image_height').value
        self.fps = self.get_parameter('fps').value
        self.camera_info_file = self.get_parameter('camera_info_file').value
        self.auto_exposure = self.get_parameter('auto_exposure').value
        self.brightness = self.get_parameter('brightness').value
        self.contrast = self.get_parameter('contrast').value
        
        # Publishers
        self.image_pub = self.create_publisher(Image, 'image_raw', 10)
        self.camera_info_pub = self.create_publisher(CameraInfo, 'camera_info', 10)
        
        # CV Bridge para conversão de imagens
        self.bridge = CvBridge()
        
        # Camera Info Manager
        self.camera_info_manager = CameraInfoManager(self, 'camera')
        if self.camera_info_file:
            self.camera_info_manager.loadCameraInfo(self.camera_info_file)
        
        # Inicializar câmera
        self.cap = None
        self.initialize_camera()
        
        # Timer para captura de imagens
        timer_period = 1.0 / self.fps
        self.timer = self.create_timer(timer_period, self.capture_and_publish)
        
        self.get_logger().info(f'Camera node iniciado - Índice: {self.camera_index}, '
                              f'Resolução: {self.image_width}x{self.image_height}, FPS: {self.fps}')

    def initialize_camera(self):
        """Inicializa a câmera com as configurações especificadas."""
        try:
            self.cap = cv2.VideoCapture(self.camera_index)
            
            if not self.cap.isOpened():
                self.get_logger().error(f'Não foi possível abrir a câmera no índice {self.camera_index}')
                return False
            
            # Configurar propriedades da câmera
            self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, self.image_width)
            self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self.image_height)
            self.cap.set(cv2.CAP_PROP_FPS, self.fps)
            
            # Configurações de exposição e brilho
            if not self.auto_exposure:
                self.cap.set(cv2.CAP_PROP_AUTO_EXPOSURE, 0.25)  # Manual exposure
            else:
                self.cap.set(cv2.CAP_PROP_AUTO_EXPOSURE, 0.75)  # Auto exposure
                
            self.cap.set(cv2.CAP_PROP_BRIGHTNESS, self.brightness / 100.0)
            self.cap.set(cv2.CAP_PROP_CONTRAST, self.contrast / 100.0)
            
            # Verificar configurações aplicadas
            actual_width = self.cap.get(cv2.CAP_PROP_FRAME_WIDTH)
            actual_height = self.cap.get(cv2.CAP_PROP_FRAME_HEIGHT)
            actual_fps = self.cap.get(cv2.CAP_PROP_FPS)
            
            self.get_logger().info(f'Câmera configurada: {actual_width}x{actual_height} @ {actual_fps} FPS')
            return True
            
        except Exception as e:
            self.get_logger().error(f'Erro ao inicializar câmera: {e}')
            return False

    def capture_and_publish(self):
        """Captura uma imagem e publica via ROS2."""
        if self.cap is None or not self.cap.isOpened():
            self.get_logger().warn('Câmera não está disponível')
            return
        
        try:
            ret, frame = self.cap.read()
            
            if not ret:
                self.get_logger().warn('Falha ao capturar frame da câmera')
                return
            
            # Timestamp atual
            current_time = self.get_clock().now().to_msg()
            
            # Converter frame OpenCV para mensagem ROS2
            image_msg = self.bridge.cv2_to_imgmsg(frame, encoding='bgr8')
            image_msg.header.stamp = current_time
            image_msg.header.frame_id = self.frame_id
            
            # Publicar imagem
            self.image_pub.publish(image_msg)
            
            # Publicar informações da câmera
            camera_info_msg = self.camera_info_manager.getCameraInfo()
            camera_info_msg.header.stamp = current_time
            camera_info_msg.header.frame_id = self.frame_id
            camera_info_msg.width = frame.shape[1]
            camera_info_msg.height = frame.shape[0]
            
            self.camera_info_pub.publish(camera_info_msg)
            
        except Exception as e:
            self.get_logger().error(f'Erro ao capturar/publicar imagem: {e}')

    def destroy_node(self):
        """Cleanup ao destruir o nó."""
        if self.cap is not None:
            self.cap.release()
        super().destroy_node()


def main(args=None):
    """Função principal."""
    rclpy.init(args=args)
    
    try:
        camera_node = CameraNode()
        rclpy.spin(camera_node)
    except KeyboardInterrupt:
        pass
    except Exception as e:
        print(f'Erro no camera_node: {e}')
    finally:
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
