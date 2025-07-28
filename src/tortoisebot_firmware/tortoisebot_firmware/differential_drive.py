#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Int32, Bool
import time
from math import pi

try:
    import RPi.GPIO as GPIO
    RPI_AVAILABLE = True
except ImportError:
    RPI_AVAILABLE = False
    print("RPi.GPIO não disponível. Executando em modo simulação.")

class DifferentialDriveController(Node):
    """
    Controlador de tração diferencial para TortoiseBot
    Adaptado para seus componentes específicos
    """
    
    def __init__(self):
        super().__init__('differential_drive_controller')
        
        # Configurações do hardware baseadas em seus componentes
        # Driver Motor Ponte H Duplo 3V-14V 5A - Motores Traseiros
        self.left_motor_pins = {
            'forward': 23,   # GPIO 23 (Pino 16) - I1
            'backward': 24,  # GPIO 24 (Pino 18) - I2  
            'enable': 12     # GPIO 12 (PWM) - Será configurado separadamente
        }
        
        self.right_motor_pins = {
            'forward': 5,    # GPIO 5 (Pino 29) - I3
            'backward': 6,   # GPIO 6 (Pino 31) - I4
            'enable': 13     # GPIO 13 (PWM) - Será configurado separadamente
        }
        
        # Parâmetros físicos do robô
        self.wheel_diameter = 0.065      # 65mm - baseado em rodas típicas 4WD
        self.wheel_separation = 0.17     # 170mm - distância entre rodas
        self.max_rpm = 60                # RPM máximo dos motores DC 3-6V
        self.max_pwm_value = 100         # PWM máximo
        self.min_pwm_value = 15          # PWM mínimo para movimento
        
        # Cálculos cinemáticos
        self.wheel_radius = self.wheel_diameter / 2
        self.wheel_circumference = 2 * pi * self.wheel_radius
        self.max_speed = (self.wheel_circumference * self.max_rpm) / 60  # m/s
        
        # Inicializar GPIO apenas se disponível
        if RPI_AVAILABLE:
            self.setup_gpio()
        
        # ROS2 subscribers e publishers
        self.cmd_vel_subscriber = self.create_subscription(
            Twist,
            'cmd_vel',
            self.cmd_vel_callback,
            10
        )
        
        # Publishers para debug
        self.left_pwm_pub = self.create_publisher(Int32, 'left_motor_pwm', 10)
        self.right_pwm_pub = self.create_publisher(Int32, 'right_motor_pwm', 10)
        self.left_dir_pub = self.create_publisher(Bool, 'left_motor_direction', 10)
        self.right_dir_pub = self.create_publisher(Bool, 'right_motor_direction', 10)
        
        # Timer para atualização periódica
        self.timer = self.create_timer(0.1, self.update_motors)
        
        # Variáveis de controle
        self.target_left_speed = 0.0
        self.target_right_speed = 0.0
        self.current_left_pwm = 0
        self.current_right_pwm = 0
        
        self.get_logger().info('Controlador Diferencial TortoiseBot iniciado')
        self.get_logger().info(f'Configuração do robô:')
        self.get_logger().info(f'  Diâmetro da roda: {self.wheel_diameter}m')
        self.get_logger().info(f'  Separação das rodas: {self.wheel_separation}m')
        self.get_logger().info(f'  RPM máximo: {self.max_rpm}')
        self.get_logger().info(f'  Velocidade máxima: {self.max_speed:.2f} m/s')
        
    def setup_gpio(self):
        """Configurar pinos GPIO para controle dos motores"""
        try:
            GPIO.setmode(GPIO.BCM)
            GPIO.setwarnings(False)
            
            # Configurar pinos dos motores traseiros
            # Motor esquerdo
            GPIO.setup(self.left_motor_pins['forward'], GPIO.OUT)
            GPIO.setup(self.left_motor_pins['backward'], GPIO.OUT)
            GPIO.setup(self.left_motor_pins['enable'], GPIO.OUT)
            
            # Motor direito  
            GPIO.setup(self.right_motor_pins['forward'], GPIO.OUT)
            GPIO.setup(self.right_motor_pins['backward'], GPIO.OUT)
            GPIO.setup(self.right_motor_pins['enable'], GPIO.OUT)
            
            # Configurar PWM nos pinos de enable
            self.left_pwm = GPIO.PWM(self.left_motor_pins['enable'], 1000)  # 1kHz
            self.right_pwm = GPIO.PWM(self.right_motor_pins['enable'], 1000)
            
            # Iniciar PWM com duty cycle 0
            self.left_pwm.start(0)
            self.right_pwm.start(0)
            
            # Parar motores inicialmente
            self.stop_motors()
            
            self.get_logger().info('GPIO configurado com sucesso')
            
        except Exception as e:
            self.get_logger().error(f'Erro ao configurar GPIO: {e}')
            
    def cmd_vel_callback(self, msg):
        """Callback para comandos de velocidade"""
        linear_vel = msg.linear.x    # m/s
        angular_vel = msg.angular.z  # rad/s
        
        # Cinemática diferencial
        # v_left = linear_vel - (angular_vel * wheel_separation / 2)
        # v_right = linear_vel + (angular_vel * wheel_separation / 2)
        
        self.target_left_speed = linear_vel - (angular_vel * self.wheel_separation / 2)
        self.target_right_speed = linear_vel + (angular_vel * self.wheel_separation / 2)
        
        # Log dos comandos recebidos
        self.get_logger().debug(
            f'Cmd recebido - Linear: {linear_vel:.2f} m/s, '
            f'Angular: {angular_vel:.2f} rad/s'
        )
        
    def velocity_to_pwm(self, velocity):
        """Converter velocidade (m/s) para valor PWM"""
        if abs(velocity) < 0.01:  # Zona morta
            return 0
            
        # Normalizar velocidade
        normalized_velocity = abs(velocity) / self.max_speed
        
        # Mapear para faixa PWM
        pwm_range = self.max_pwm_value - self.min_pwm_value
        pwm_value = self.min_pwm_value + (normalized_velocity * pwm_range)
        
        # Limitar PWM
        pwm_value = max(0, min(self.max_pwm_value, pwm_value))
        
        return int(pwm_value)
        
    def set_motor_speed(self, motor_side, speed):
        """Configurar velocidade e direção do motor"""
        if not RPI_AVAILABLE:
            return
            
        pwm_value = self.velocity_to_pwm(speed)
        direction_forward = speed >= 0
        
        if motor_side == 'left':
            pins = self.left_motor_pins
            pwm_controller = self.left_pwm
        else:
            pins = self.right_motor_pins
            pwm_controller = self.right_pwm
            
        try:
            if pwm_value == 0:
                # Parar motor
                GPIO.output(pins['forward'], GPIO.LOW)
                GPIO.output(pins['backward'], GPIO.LOW)
                pwm_controller.ChangeDutyCycle(0)
            else:
                if direction_forward:
                    GPIO.output(pins['forward'], GPIO.HIGH)
                    GPIO.output(pins['backward'], GPIO.LOW)
                else:
                    GPIO.output(pins['forward'], GPIO.LOW)
                    GPIO.output(pins['backward'], GPIO.HIGH)
                    
                pwm_controller.ChangeDutyCycle(pwm_value)
                
        except Exception as e:
            self.get_logger().error(f'Erro ao configurar motor {motor_side}: {e}')
            
    def update_motors(self):
        """Atualização periódica dos motores"""
        # Configurar motores
        self.set_motor_speed('left', self.target_left_speed)
        self.set_motor_speed('right', self.target_right_speed)
        
        # Calcular PWM atual para debug
        self.current_left_pwm = self.velocity_to_pwm(self.target_left_speed)
        self.current_right_pwm = self.velocity_to_pwm(self.target_right_speed)
        
        # Publicar informações de debug
        left_pwm_msg = Int32()
        left_pwm_msg.data = self.current_left_pwm
        self.left_pwm_pub.publish(left_pwm_msg)
        
        right_pwm_msg = Int32()
        right_pwm_msg.data = self.current_right_pwm
        self.right_pwm_pub.publish(right_pwm_msg)
        
        left_dir_msg = Bool()
        left_dir_msg.data = self.target_left_speed >= 0
        self.left_dir_pub.publish(left_dir_msg)
        
        right_dir_msg = Bool()
        right_dir_msg.data = self.target_right_speed >= 0
        self.right_dir_pub.publish(right_dir_msg)
        
    def stop_motors(self):
        """Parar todos os motores"""
        if not RPI_AVAILABLE:
            return
            
        try:
            # Motor esquerdo
            GPIO.output(self.left_motor_pins['forward'], GPIO.LOW)
            GPIO.output(self.left_motor_pins['backward'], GPIO.LOW)
            self.left_pwm.ChangeDutyCycle(0)
            
            # Motor direito
            GPIO.output(self.right_motor_pins['forward'], GPIO.LOW)
            GPIO.output(self.right_motor_pins['backward'], GPIO.LOW)
            self.right_pwm.ChangeDutyCycle(0)
            
        except Exception as e:
            self.get_logger().error(f'Erro ao parar motores: {e}')
            
    def cleanup(self):
        """Limpeza dos recursos GPIO"""
        if RPI_AVAILABLE:
            self.stop_motors()
            self.left_pwm.stop()
            self.right_pwm.stop()
            GPIO.cleanup()
            self.get_logger().info('GPIO limpo')

def main(args=None):
    rclpy.init(args=args)
    
    try:
        controller = DifferentialDriveController()
        rclpy.spin(controller)
    except KeyboardInterrupt:
        pass
    finally:
        if 'controller' in locals():
            controller.cleanup()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
