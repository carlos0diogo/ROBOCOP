#!/usr/bin/env python3
"""
TortoiseBot Camera Calibration Node
Utilitário para calibração da webcam Lenovo 300
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np
import os
import yaml


class CameraCalibrationNode(Node):
    """Nó para calibração automática da câmera."""
    
    def __init__(self):
        super().__init__('camera_calibration_node')
        
        # Parâmetros
        self.declare_parameter('camera_index', 0)
        self.declare_parameter('chessboard_size', [9, 6])  # Padrão xadrez interno
        self.declare_parameter('square_size', 0.025)  # 2.5cm por quadrado
        self.declare_parameter('output_file', 'camera_calibration.yaml')
        self.declare_parameter('samples_needed', 20)
        
        # Obter parâmetros
        self.camera_index = self.get_parameter('camera_index').value
        self.chessboard_size = tuple(self.get_parameter('chessboard_size').value)
        self.square_size = self.get_parameter('square_size').value
        self.output_file = self.get_parameter('output_file').value
        self.samples_needed = self.get_parameter('samples_needed').value
        
        # CV Bridge
        self.bridge = CvBridge()
        
        # Preparação para calibração
        self.criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)
        
        # Preparar pontos 3D do padrão
        self.objp = np.zeros((self.chessboard_size[0] * self.chessboard_size[1], 3), np.float32)
        self.objp[:, :2] = np.mgrid[0:self.chessboard_size[0], 0:self.chessboard_size[1]].T.reshape(-1, 2)
        self.objp *= self.square_size
        
        # Arrays para armazenar pontos
        self.objpoints = []  # Pontos 3D no mundo real
        self.imgpoints = []  # Pontos 2D na imagem
        
        # Estado da calibração
        self.samples_collected = 0
        self.cap = None
        
        # Inicializar câmera
        self.initialize_camera()
        
        # Timer para captura
        self.timer = self.create_timer(1.0, self.capture_calibration_frame)
        
        self.get_logger().info(f'Iniciando calibração da câmera - {self.samples_needed} amostras necessárias')
        self.get_logger().info(f'Padrão de calibração: {self.chessboard_size[0]}x{self.chessboard_size[1]} quadrados')

    def initialize_camera(self):
        """Inicializa a câmera."""
        try:
            self.cap = cv2.VideoCapture(self.camera_index)
            
            if not self.cap.isOpened():
                self.get_logger().error(f'Não foi possível abrir a câmera no índice {self.camera_index}')
                return False
                
            self.get_logger().info('Câmera inicializada para calibração')
            return True
            
        except Exception as e:
            self.get_logger().error(f'Erro ao inicializar câmera: {e}')
            return False

    def capture_calibration_frame(self):
        """Captura frame e processa para calibração."""
        if self.cap is None or not self.cap.isOpened():
            return
            
        if self.samples_collected >= self.samples_needed:
            self.perform_calibration()
            return
            
        try:
            ret, frame = self.cap.read()
            
            if not ret:
                self.get_logger().warn('Falha ao capturar frame')
                return
            
            gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
            
            # Encontrar cantos do padrão de xadrez
            ret, corners = cv2.findChessboardCorners(gray, self.chessboard_size, None)
            
            if ret:
                # Refinar coordenadas dos cantos
                corners2 = cv2.cornerSubPix(gray, corners, (11, 11), (-1, -1), self.criteria)
                
                # Armazenar pontos
                self.objpoints.append(self.objp)
                self.imgpoints.append(corners2)
                
                self.samples_collected += 1
                
                self.get_logger().info(f'Amostra {self.samples_collected}/{self.samples_needed} coletada')
                
                # Desenhar cantos na imagem (para debug)
                cv2.drawChessboardCorners(frame, self.chessboard_size, corners2, ret)
                
                # Mostrar imagem (opcional - só em debug)
                # cv2.imshow('Calibration', frame)
                # cv2.waitKey(500)
            else:
                self.get_logger().debug('Padrão de xadrez não encontrado no frame')
                
        except Exception as e:
            self.get_logger().error(f'Erro ao processar frame de calibração: {e}')

    def perform_calibration(self):
        """Executa a calibração e salva os resultados."""
        if len(self.objpoints) < self.samples_needed:
            self.get_logger().error('Amostras insuficientes para calibração')
            return
            
        try:
            # Obter dimensões da imagem
            ret, frame = self.cap.read()
            if not ret:
                self.get_logger().error('Não foi possível obter dimensões da imagem')
                return
                
            gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
            img_shape = gray.shape[::-1]
            
            # Calibrar câmera
            ret, camera_matrix, dist_coeffs, rvecs, tvecs = cv2.calibrateCamera(
                self.objpoints, self.imgpoints, img_shape, None, None
            )
            
            if ret:
                # Calcular erro de reprojeção
                mean_error = 0
                for i in range(len(self.objpoints)):
                    imgpoints2, _ = cv2.projectPoints(
                        self.objpoints[i], rvecs[i], tvecs[i], camera_matrix, dist_coeffs
                    )
                    error = cv2.norm(self.imgpoints[i], imgpoints2, cv2.NORM_L2) / len(imgpoints2)
                    mean_error += error
                    
                mean_error /= len(self.objpoints)
                
                self.get_logger().info(f'Calibração concluída! Erro médio de reprojeção: {mean_error:.4f}')
                
                # Salvar resultados
                self.save_calibration_data(camera_matrix, dist_coeffs, img_shape, mean_error)
                
                # Parar timer
                self.timer.cancel()
                
            else:
                self.get_logger().error('Falha na calibração da câmera')
                
        except Exception as e:
            self.get_logger().error(f'Erro durante calibração: {e}')

    def save_calibration_data(self, camera_matrix, dist_coeffs, img_shape, mean_error):
        """Salva dados de calibração em arquivo YAML."""
        calibration_data = {
            'image_width': int(img_shape[0]),
            'image_height': int(img_shape[1]),
            'camera_name': 'lenovo_300_camera',
            'camera_matrix': {
                'rows': 3,
                'cols': 3,
                'data': camera_matrix.flatten().tolist()
            },
            'distortion_model': 'plumb_bob',
            'distortion_coefficients': {
                'rows': 1,
                'cols': 5,
                'data': dist_coeffs.flatten().tolist()
            },
            'rectification_matrix': {
                'rows': 3,
                'cols': 3,
                'data': np.eye(3).flatten().tolist()
            },
            'projection_matrix': {
                'rows': 3,
                'cols': 4,
                'data': np.hstack([camera_matrix, np.zeros((3, 1))]).flatten().tolist()
            },
            'reprojection_error': float(mean_error)
        }
        
        try:
            with open(self.output_file, 'w') as f:
                yaml.dump(calibration_data, f, default_flow_style=False)
                
            self.get_logger().info(f'Dados de calibração salvos em: {self.output_file}')
            
        except Exception as e:
            self.get_logger().error(f'Erro ao salvar calibração: {e}')

    def destroy_node(self):
        """Cleanup ao destruir o nó."""
        if self.cap is not None:
            self.cap.release()
        cv2.destroyAllWindows()
        super().destroy_node()


def main(args=None):
    """Função principal."""
    rclpy.init(args=args)
    
    try:
        calibration_node = CameraCalibrationNode()
        rclpy.spin(calibration_node)
    except KeyboardInterrupt:
        pass
    except Exception as e:
        print(f'Erro no camera_calibration: {e}')
    finally:
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
