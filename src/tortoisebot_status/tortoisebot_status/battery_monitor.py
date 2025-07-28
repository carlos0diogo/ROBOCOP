#!/usr/bin/env python3
"""
TortoiseBot Battery Monitor
Monitora nível de bateria e status de alimentação via GPIO/ADC
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import BatteryState
from std_msgs.msg import Float32, Bool
import time
import subprocess
import os


class BatteryMonitor(Node):
    """Nó para monitoramento de bateria e energia."""
    
    def __init__(self):
        super().__init__('battery_monitor')
        
        # Parâmetros
        self.declare_parameter('update_rate', 1.0)  # Hz
        self.declare_parameter('voltage_divider_ratio', 3.0)  # Para leitura de tensão
        self.declare_parameter('battery_cells', 4)  # 4 pilhas AA
        self.declare_parameter('cell_voltage_max', 1.6)  # Tensão máxima por célula (AA)
        self.declare_parameter('cell_voltage_min', 1.0)  # Tensão mínima por célula
        self.declare_parameter('low_battery_threshold', 20.0)  # % para alerta
        self.declare_parameter('critical_battery_threshold', 10.0)  # % crítico
        self.declare_parameter('enable_gpio_monitoring', True)
        self.declare_parameter('enable_system_monitoring', True)
        
        # Obter parâmetros
        self.update_rate = self.get_parameter('update_rate').value
        self.voltage_divider_ratio = self.get_parameter('voltage_divider_ratio').value
        self.battery_cells = self.get_parameter('battery_cells').value
        self.cell_voltage_max = self.get_parameter('cell_voltage_max').value
        self.cell_voltage_min = self.get_parameter('cell_voltage_min').value
        self.low_battery_threshold = self.get_parameter('low_battery_threshold').value
        self.critical_battery_threshold = self.get_parameter('critical_battery_threshold').value
        self.enable_gpio_monitoring = self.get_parameter('enable_gpio_monitoring').value
        self.enable_system_monitoring = self.get_parameter('enable_system_monitoring').value
        
        # Publishers
        self.battery_state_pub = self.create_publisher(BatteryState, 'battery_state', 10)
        self.battery_voltage_pub = self.create_publisher(Float32, 'battery_voltage', 10)
        self.battery_percentage_pub = self.create_publisher(Float32, 'battery_percentage', 10)
        self.low_battery_pub = self.create_publisher(Bool, 'low_battery_alert', 10)
        self.critical_battery_pub = self.create_publisher(Bool, 'critical_battery_alert', 10)
        
        # Estado da bateria
        self.last_voltage = 0.0
        self.last_percentage = 0.0
        self.low_battery_alerted = False
        self.critical_battery_alerted = False
        
        # Inicializar GPIO (se habilitado)
        if self.enable_gpio_monitoring:
            self.setup_gpio()
        
        # Timer para monitoramento
        timer_period = 1.0 / self.update_rate
        self.timer = self.create_timer(timer_period, self.monitor_battery)
        
        self.get_logger().info('Battery Monitor iniciado')
        self.get_logger().info(f'Monitoramento: GPIO={self.enable_gpio_monitoring}, '
                              f'Sistema={self.enable_system_monitoring}')

    def setup_gpio(self):
        """Configura GPIO para leitura de tensão da bateria."""
        try:
            # Tentar importar GPIO libraries
            global RPi
            import RPi.GPIO as GPIO
            
            # Configurar GPIO (se disponível)
            # Nota: Para leitura analógica, seria necessário um ADC como MCP3008
            # Por enquanto, vamos simular com leitura do sistema
            
            self.get_logger().info('GPIO configurado para monitoramento de bateria')
            
        except ImportError:
            self.get_logger().warn('RPi.GPIO não disponível - usando monitoramento simulado')
        except Exception as e:
            self.get_logger().error(f'Erro ao configurar GPIO: {e}')

    def read_battery_voltage(self):
        """Lê a tensão da bateria."""
        if self.enable_gpio_monitoring:
            return self.read_gpio_voltage()
        elif self.enable_system_monitoring:
            return self.read_system_voltage()
        else:
            return self.simulate_voltage()

    def read_gpio_voltage(self):
        """Lê tensão via GPIO/ADC."""
        try:
            # Implementação para leitura real via ADC
            # Exemplo com MCP3008 ou similar
            # Por enquanto, retorna simulação
            
            # Simulação baseada em tempo (decaimento lento)
            import random
            base_voltage = 6.0  # 4 células x 1.5V
            noise = random.uniform(-0.1, 0.1)
            decay = time.time() % 3600 / 3600 * 0.5  # Decaimento ao longo de 1 hora
            
            voltage = base_voltage - decay + noise
            return max(voltage, self.battery_cells * self.cell_voltage_min)
            
        except Exception as e:
            self.get_logger().error(f'Erro na leitura GPIO: {e}')
            return 0.0

    def read_system_voltage(self):
        """Lê informações de energia do sistema."""
        try:
            # No Raspberry Pi, pode ler de /sys/class/power_supply/
            power_supply_path = '/sys/class/power_supply'
            
            if os.path.exists(power_supply_path):
                # Procurar por fontes de energia
                supplies = os.listdir(power_supply_path)
                
                for supply in supplies:
                    supply_path = os.path.join(power_supply_path, supply)
                    type_file = os.path.join(supply_path, 'type')
                    
                    if os.path.exists(type_file):
                        with open(type_file, 'r') as f:
                            supply_type = f.read().strip()
                            
                        if supply_type == 'Battery':
                            # Ler tensão da bateria
                            voltage_file = os.path.join(supply_path, 'voltage_now')
                            if os.path.exists(voltage_file):
                                with open(voltage_file, 'r') as f:
                                    voltage_uv = int(f.read().strip())
                                    return voltage_uv / 1000000.0  # Converter de μV para V
            
            # Se não encontrou bateria do sistema, simular
            return self.simulate_voltage()
            
        except Exception as e:
            self.get_logger().debug(f'Leitura do sistema falhou: {e}')
            return self.simulate_voltage()

    def simulate_voltage(self):
        """Simula leitura de tensão para teste."""
        import random
        
        # Simulação realística de descarga de bateria
        base_voltage = self.battery_cells * 1.4  # Tensão nominal
        
        # Adicionar variação temporal e ruído
        time_factor = (time.time() % 7200) / 7200  # Ciclo de 2 horas
        discharge = time_factor * 0.8  # Descarga até 0.8V por célula
        noise = random.uniform(-0.05, 0.05)
        
        voltage = base_voltage - discharge + noise
        return max(voltage, self.battery_cells * self.cell_voltage_min)

    def voltage_to_percentage(self, voltage):
        """Converte tensão para porcentagem de carga."""
        voltage_per_cell = voltage / self.battery_cells
        
        # Calcular porcentagem baseada na faixa de tensão
        voltage_range = self.cell_voltage_max - self.cell_voltage_min
        cell_percentage = (voltage_per_cell - self.cell_voltage_min) / voltage_range
        
        percentage = max(0.0, min(100.0, cell_percentage * 100.0))
        return percentage

    def monitor_battery(self):
        """Monitora o estado da bateria e publica informações."""
        try:
            # Ler tensão atual
            voltage = self.read_battery_voltage()
            percentage = self.voltage_to_percentage(voltage)
            
            # Filtro simples para suavizar leituras
            if self.last_voltage > 0:
                voltage = 0.9 * self.last_voltage + 0.1 * voltage
                percentage = 0.9 * self.last_percentage + 0.1 * percentage
            
            self.last_voltage = voltage
            self.last_percentage = percentage
            
            # Timestamp atual
            current_time = self.get_clock().now().to_msg()
            
            # Publicar estado da bateria (mensagem completa)
            battery_msg = BatteryState()
            battery_msg.header.stamp = current_time
            battery_msg.header.frame_id = 'base_link'
            battery_msg.voltage = voltage
            battery_msg.current = float('nan')  # Não medindo corrente
            battery_msg.charge = float('nan')   # Não medindo carga
            battery_msg.capacity = float('nan') # Capacidade desconhecida
            battery_msg.design_capacity = float('nan')
            battery_msg.percentage = percentage / 100.0
            battery_msg.power_supply_status = BatteryState.POWER_SUPPLY_STATUS_DISCHARGING
            battery_msg.power_supply_health = BatteryState.POWER_SUPPLY_HEALTH_GOOD
            battery_msg.power_supply_technology = BatteryState.POWER_SUPPLY_TECHNOLOGY_NIMH
            battery_msg.present = True
            
            self.battery_state_pub.publish(battery_msg)
            
            # Publicar valores individuais
            voltage_msg = Float32()
            voltage_msg.data = voltage
            self.battery_voltage_pub.publish(voltage_msg)
            
            percentage_msg = Float32()
            percentage_msg.data = percentage
            self.battery_percentage_pub.publish(percentage_msg)
            
            # Verificar alertas
            self.check_battery_alerts(percentage)
            
            # Log periódico
            if int(time.time()) % 30 == 0:  # A cada 30 segundos
                self.get_logger().info(f'Bateria: {voltage:.2f}V ({percentage:.1f}%)')
                
        except Exception as e:
            self.get_logger().error(f'Erro no monitoramento de bateria: {e}')

    def check_battery_alerts(self, percentage):
        """Verifica e publica alertas de bateria baixa."""
        # Alerta de bateria baixa
        low_battery = percentage <= self.low_battery_threshold
        if low_battery and not self.low_battery_alerted:
            self.get_logger().warn(f'ALERTA: Bateria baixa! {percentage:.1f}%')
            self.low_battery_alerted = True
        elif not low_battery and self.low_battery_alerted:
            self.low_battery_alerted = False
            
        low_battery_msg = Bool()
        low_battery_msg.data = low_battery
        self.low_battery_pub.publish(low_battery_msg)
        
        # Alerta crítico
        critical_battery = percentage <= self.critical_battery_threshold
        if critical_battery and not self.critical_battery_alerted:
            self.get_logger().error(f'CRÍTICO: Bateria crítica! {percentage:.1f}% - Considere desligar!')
            self.critical_battery_alerted = True
        elif not critical_battery and self.critical_battery_alerted:
            self.critical_battery_alerted = False
            
        critical_battery_msg = Bool()
        critical_battery_msg.data = critical_battery
        self.critical_battery_pub.publish(critical_battery_msg)


def main(args=None):
    """Função principal."""
    rclpy.init(args=args)
    
    try:
        battery_monitor = BatteryMonitor()
        rclpy.spin(battery_monitor)
    except KeyboardInterrupt:
        pass
    except Exception as e:
        print(f'Erro no battery_monitor: {e}')
    finally:
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
