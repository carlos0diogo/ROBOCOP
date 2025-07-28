#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
import time
from math import pi

try:
    import RPi.GPIO as GPIO
    RPI_AVAILABLE = True
except ImportError:
    RPI_AVAILABLE = False
    print("RPi.GPIO não disponível. Executando em modo simulação.")

class UltrasonicSensorNode(Node):
    """
    Nó para sensor ultrassônico HC-SR04
    Publica dados compatíveis com LaserScan para integração com Nav2
    """
    
    def __init__(self):
        super().__init__('ultrasonic_sensor')
        
        # Configuração dos pinos HC-SR04
        self.trigger_pin = 7   # GPIO 7 (Pino 26)
        self.echo_pin = 8      # GPIO 8 (Pino 24)
        
        # Parâmetros do sensor
        self.max_range = 4.0   # Alcance máximo em metros
        self.min_range = 0.02  # Alcance mínimo em metros
        self.angle_range = 0.52  # ~30 graus em radianos
        
        # Configurar GPIO
        if RPI_AVAILABLE:
            self.setup_gpio()
        
        # Publisher para dados do sensor
        self.scan_publisher = self.create_publisher(
            LaserScan,
            'front_ultrasonic/scan',
            10
        )
        
        # Timer para leituras periódicas
        self.timer = self.create_timer(0.2, self.read_sensor)  # 5Hz
        
        self.get_logger().info('Sensor Ultrassônico HC-SR04 iniciado')
        
    def setup_gpio(self):
        """Configurar pinos GPIO para o sensor ultrassônico"""
        try:
            GPIO.setmode(GPIO.BCM)
            GPIO.setwarnings(False)
            
            GPIO.setup(self.trigger_pin, GPIO.OUT)
            GPIO.setup(self.echo_pin, GPIO.IN)
            
            # Garantir que trigger inicie em LOW
            GPIO.output(self.trigger_pin, GPIO.LOW)
            time.sleep(0.1)
            
            self.get_logger().info('GPIO configurado para sensor ultrassônico')
            
        except Exception as e:
            self.get_logger().error(f'Erro ao configurar GPIO: {e}')
            
    def read_distance(self):
        """Ler distância do sensor HC-SR04"""
        if not RPI_AVAILABLE:
            # Simular leitura para teste
            import random
            return random.uniform(0.3, 3.0)
            
        try:
            # Enviar pulso trigger
            GPIO.output(self.trigger_pin, GPIO.HIGH)
            time.sleep(0.00001)  # 10 microsegundos
            GPIO.output(self.trigger_pin, GPIO.LOW)
            
            # Medir tempo do pulso echo
            timeout = time.time() + 0.1  # Timeout de 100ms
            
            # Aguardar início do pulso
            pulse_start = time.time()
            while GPIO.input(self.echo_pin) == 0:
                pulse_start = time.time()
                if pulse_start > timeout:
                    return None
                    
            # Aguardar fim do pulso
            pulse_end = time.time()
            while GPIO.input(self.echo_pin) == 1:
                pulse_end = time.time()
                if pulse_end > timeout:
                    return None
                    
            # Calcular distância
            pulse_duration = pulse_end - pulse_start
            distance = (pulse_duration * 34300) / 2  # Velocidade do som = 343 m/s
            distance = distance / 100  # Converter cm para metros
            
            # Filtrar leituras inválidas
            if distance < self.min_range or distance > self.max_range:
                return None
                
            return distance
            
        except Exception as e:
            self.get_logger().error(f'Erro ao ler sensor: {e}')
            return None
            
    def read_sensor(self):
        """Callback para leitura periódica do sensor"""
        distance = self.read_distance()
        
        # Criar mensagem LaserScan
        scan_msg = LaserScan()
        scan_msg.header.stamp = self.get_clock().now().to_msg()
        scan_msg.header.frame_id = 'front_ultrasonic_link'
        
        # Configurar parâmetros do scan
        scan_msg.angle_min = -self.angle_range / 2
        scan_msg.angle_max = self.angle_range / 2
        scan_msg.angle_increment = self.angle_range / 4  # 5 raios
        scan_msg.time_increment = 0.0
        scan_msg.scan_time = 0.2
        scan_msg.range_min = self.min_range
        scan_msg.range_max = self.max_range
        
        # Preencher dados dos raios
        if distance is not None:
            # Usar a mesma distância para todos os raios (sensor não direcional)
            scan_msg.ranges = [distance] * 5
            scan_msg.intensities = [1.0] * 5
        else:
            # Sem detecção válida
            scan_msg.ranges = [float('inf')] * 5
            scan_msg.intensities = [0.0] * 5
            
        # Publicar scan
        self.scan_publisher.publish(scan_msg)
        
        # Log opcional para debug
        if distance is not None:
            self.get_logger().debug(f'Distância ultrassônica: {distance:.2f}m')
        else:
            self.get_logger().debug('Nenhuma detecção válida')
            
    def cleanup(self):
        """Limpeza dos recursos GPIO"""
        if RPI_AVAILABLE:
            GPIO.cleanup()
            self.get_logger().info('GPIO limpo')

def main(args=None):
    rclpy.init(args=args)
    
    try:
        sensor_node = UltrasonicSensorNode()
        rclpy.spin(sensor_node)
    except KeyboardInterrupt:
        pass
    finally:
        if 'sensor_node' in locals():
            sensor_node.cleanup()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
