#!/usr/bin/env python3
"""
TortoiseBot System Monitor
Monitora CPU, temperatura, memória e status geral do sistema
"""

import rclpy
from rclpy.node import Node
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from std_msgs.msg import Float32, String
import psutil
import subprocess
import os
import time


class SystemMonitor(Node):
    """Nó para monitoramento geral do sistema."""
    
    def __init__(self):
        super().__init__('system_monitor')
        
        # Parâmetros
        self.declare_parameter('update_rate', 0.5)  # Hz
        self.declare_parameter('cpu_temp_warning', 70.0)  # °C
        self.declare_parameter('cpu_temp_critical', 80.0)  # °C
        self.declare_parameter('memory_warning', 80.0)  # %
        self.declare_parameter('memory_critical', 90.0)  # %
        self.declare_parameter('disk_warning', 80.0)  # %
        self.declare_parameter('disk_critical', 90.0)  # %
        self.declare_parameter('enable_detailed_logging', False)
        
        # Obter parâmetros
        self.update_rate = self.get_parameter('update_rate').value
        self.cpu_temp_warning = self.get_parameter('cpu_temp_warning').value
        self.cpu_temp_critical = self.get_parameter('cpu_temp_critical').value
        self.memory_warning = self.get_parameter('memory_warning').value
        self.memory_critical = self.get_parameter('memory_critical').value
        self.disk_warning = self.get_parameter('disk_warning').value
        self.disk_critical = self.get_parameter('disk_critical').value
        self.detailed_logging = self.get_parameter('enable_detailed_logging').value
        
        # Publishers
        self.diagnostics_pub = self.create_publisher(DiagnosticArray, 'diagnostics', 10)
        self.cpu_temp_pub = self.create_publisher(Float32, 'cpu_temperature', 10)
        self.cpu_usage_pub = self.create_publisher(Float32, 'cpu_usage', 10)
        self.memory_usage_pub = self.create_publisher(Float32, 'memory_usage', 10)
        self.disk_usage_pub = self.create_publisher(Float32, 'disk_usage', 10)
        self.system_status_pub = self.create_publisher(String, 'system_status', 10)
        
        # Estado do sistema
        self.last_diagnostics = {}
        
        # Timer para monitoramento
        timer_period = 1.0 / self.update_rate
        self.timer = self.create_timer(timer_period, self.monitor_system)
        
        self.get_logger().info('System Monitor iniciado')
        self.get_logger().info(f'Limites: CPU temp {self.cpu_temp_warning}°C/{self.cpu_temp_critical}°C, '
                              f'Memória {self.memory_warning}%/{self.memory_critical}%')

    def get_cpu_temperature(self):
        """Obtém temperatura da CPU."""
        try:
            # Método 1: Raspberry Pi específico
            if os.path.exists('/sys/class/thermal/thermal_zone0/temp'):
                with open('/sys/class/thermal/thermal_zone0/temp', 'r', encoding='utf-8') as f:
                    temp_millidegree = int(f.read().strip())
                    return temp_millidegree / 1000.0
            
            # Método 2: Usando vcgencmd (Raspberry Pi)
            try:
                result = subprocess.run(['vcgencmd', 'measure_temp'], 
                                      capture_output=True, text=True, timeout=5)
                if result.returncode == 0:
                    temp_str = result.stdout.strip()
                    # Formato: temp=42.8'C
                    temp_value = float(temp_str.split('=')[1].replace("'C", ""))
                    return temp_value
            except (subprocess.TimeoutExpired, subprocess.CalledProcessError, FileNotFoundError):
                pass
            
            # Método 3: psutil (mais genérico)
            try:
                temps = psutil.sensors_temperatures()
                if 'cpu_thermal' in temps:
                    return temps['cpu_thermal'][0].current
                elif 'coretemp' in temps:
                    return temps['coretemp'][0].current
            except AttributeError:
                pass
            
            # Fallback: simulação baseada em carga de CPU
            cpu_percent = psutil.cpu_percent(interval=0.1)
            base_temp = 35.0  # Temperatura base
            load_temp = cpu_percent * 0.5  # Aumento por carga
            return base_temp + load_temp
            
        except Exception as e:
            self.get_logger().debug(f'Erro ao ler temperatura: {e}')
            return 40.0  # Valor padrão seguro

    def get_system_info(self):
        """Coleta informações do sistema."""
        try:
            # CPU
            cpu_percent = psutil.cpu_percent(interval=0.1)
            cpu_freq = psutil.cpu_freq()
            cpu_count = psutil.cpu_count()
            
            # Memória
            memory = psutil.virtual_memory()
            memory_percent = memory.percent
            memory_available_gb = memory.available / (1024**3)
            memory_total_gb = memory.total / (1024**3)
            
            # Disco
            disk = psutil.disk_usage('/')
            disk_percent = disk.percent
            disk_free_gb = disk.free / (1024**3)
            disk_total_gb = disk.total / (1024**3)
            
            # Temperatura
            cpu_temp = self.get_cpu_temperature()
            
            # Uptime
            boot_time = psutil.boot_time()
            uptime_seconds = time.time() - boot_time
            uptime_hours = uptime_seconds / 3600
            
            # Load average (Linux)
            try:
                load_avg = os.getloadavg()
            except (OSError, AttributeError):
                load_avg = (0.0, 0.0, 0.0)
            
            return {
                'cpu_percent': cpu_percent,
                'cpu_freq_mhz': cpu_freq.current if cpu_freq else 0,
                'cpu_count': cpu_count,
                'memory_percent': memory_percent,
                'memory_available_gb': memory_available_gb,
                'memory_total_gb': memory_total_gb,
                'disk_percent': disk_percent,
                'disk_free_gb': disk_free_gb,
                'disk_total_gb': disk_total_gb,
                'cpu_temp': cpu_temp,
                'uptime_hours': uptime_hours,
                'load_avg_1min': load_avg[0],
                'load_avg_5min': load_avg[1],
                'load_avg_15min': load_avg[2],
            }
            
        except Exception as e:
            self.get_logger().error(f'Erro ao coletar informações do sistema: {e}')
            return {}

    def create_diagnostic_status(self, name, level, message, key_values=None):
        """Cria uma mensagem de status de diagnóstico."""
        status = DiagnosticStatus()
        status.name = name
        status.level = level
        status.message = message
        
        if key_values:
            for key, value in key_values.items():
                kv = KeyValue()
                kv.key = key
                kv.value = str(value)
                status.values.append(kv)
        
        return status

    def monitor_system(self):
        """Monitora o sistema e publica diagnósticos."""
        try:
            # Coletar informações do sistema
            info = self.get_system_info()
            
            if not info:
                return
            
            # Timestamp atual
            current_time = self.get_clock().now().to_msg()
            
            # Publicar valores individuais
            cpu_temp_msg = Float32()
            cpu_temp_msg.data = info['cpu_temp']
            self.cpu_temp_pub.publish(cpu_temp_msg)
            
            cpu_usage_msg = Float32()
            cpu_usage_msg.data = info['cpu_percent']
            self.cpu_usage_pub.publish(cpu_usage_msg)
            
            memory_usage_msg = Float32()
            memory_usage_msg.data = info['memory_percent']
            self.memory_usage_pub.publish(memory_usage_msg)
            
            disk_usage_msg = Float32()
            disk_usage_msg.data = info['disk_percent']
            self.disk_usage_pub.publish(disk_usage_msg)
            
            # Criar array de diagnósticos
            diagnostics = DiagnosticArray()
            diagnostics.header.stamp = current_time
            diagnostics.header.frame_id = 'base_link'
            
            # Diagnóstico de CPU
            cpu_level = DiagnosticStatus.OK
            cpu_message = f"CPU: {info['cpu_percent']:.1f}% @ {info['cpu_freq_mhz']:.0f}MHz"
            
            if info['cpu_percent'] > 80:
                cpu_level = DiagnosticStatus.WARN
                cpu_message += " - Alta utilização"
            elif info['cpu_percent'] > 95:
                cpu_level = DiagnosticStatus.ERROR
                cpu_message += " - Utilização crítica"
            
            cpu_status = self.create_diagnostic_status(
                'tortoisebot/cpu',
                cpu_level,
                cpu_message,
                {
                    'usage_percent': f"{info['cpu_percent']:.1f}",
                    'frequency_mhz': f"{info['cpu_freq_mhz']:.0f}",
                    'cores': info['cpu_count'],
                    'load_avg_1min': f"{info['load_avg_1min']:.2f}",
                }
            )
            diagnostics.status.append(cpu_status)
            
            # Diagnóstico de temperatura
            temp_level = DiagnosticStatus.OK
            temp_message = f"Temperatura: {info['cpu_temp']:.1f}°C"
            
            if info['cpu_temp'] > self.cpu_temp_warning:
                temp_level = DiagnosticStatus.WARN
                temp_message += " - Alta temperatura"
            elif info['cpu_temp'] > self.cpu_temp_critical:
                temp_level = DiagnosticStatus.ERROR
                temp_message += " - Temperatura crítica"
            
            temp_status = self.create_diagnostic_status(
                'tortoisebot/temperature',
                temp_level,
                temp_message,
                {
                    'temperature_celsius': f"{info['cpu_temp']:.1f}",
                    'warning_threshold': self.cpu_temp_warning,
                    'critical_threshold': self.cpu_temp_critical,
                }
            )
            diagnostics.status.append(temp_status)
            
            # Diagnóstico de memória
            mem_level = DiagnosticStatus.OK
            mem_message = f"Memória: {info['memory_percent']:.1f}% " \
                         f"({info['memory_available_gb']:.1f}/{info['memory_total_gb']:.1f} GB)"
            
            if info['memory_percent'] > self.memory_warning:
                mem_level = DiagnosticStatus.WARN
                mem_message += " - Uso elevado"
            elif info['memory_percent'] > self.memory_critical:
                mem_level = DiagnosticStatus.ERROR
                mem_message += " - Uso crítico"
            
            mem_status = self.create_diagnostic_status(
                'tortoisebot/memory',
                mem_level,
                mem_message,
                {
                    'usage_percent': f"{info['memory_percent']:.1f}",
                    'available_gb': f"{info['memory_available_gb']:.1f}",
                    'total_gb': f"{info['memory_total_gb']:.1f}",
                }
            )
            diagnostics.status.append(mem_status)
            
            # Diagnóstico de disco
            disk_level = DiagnosticStatus.OK
            disk_message = f"Disco: {info['disk_percent']:.1f}% " \
                          f"({info['disk_free_gb']:.1f}/{info['disk_total_gb']:.1f} GB livres)"
            
            if info['disk_percent'] > self.disk_warning:
                disk_level = DiagnosticStatus.WARN
                disk_message += " - Espaço baixo"
            elif info['disk_percent'] > self.disk_critical:
                disk_level = DiagnosticStatus.ERROR
                disk_message += " - Espaço crítico"
            
            disk_status = self.create_diagnostic_status(
                'tortoisebot/disk',
                disk_level,
                disk_message,
                {
                    'usage_percent': f"{info['disk_percent']:.1f}",
                    'free_gb': f"{info['disk_free_gb']:.1f}",
                    'total_gb': f"{info['disk_total_gb']:.1f}",
                }
            )
            diagnostics.status.append(disk_status)
            
            # Diagnóstico geral do sistema
            overall_level = max([cpu_level, temp_level, mem_level, disk_level])
            overall_message = f"Sistema: Uptime {info['uptime_hours']:.1f}h"
            
            if overall_level == DiagnosticStatus.ERROR:
                overall_message += " - PROBLEMAS CRÍTICOS"
            elif overall_level == DiagnosticStatus.WARN:
                overall_message += " - Avisos ativos"
            else:
                overall_message += " - Funcionando normalmente"
            
            overall_status = self.create_diagnostic_status(
                'tortoisebot/system',
                overall_level,
                overall_message,
                {
                    'uptime_hours': f"{info['uptime_hours']:.1f}",
                    'load_avg_5min': f"{info['load_avg_5min']:.2f}",
                    'ros_node': 'active',
                }
            )
            diagnostics.status.append(overall_status)
            
            # Publicar diagnósticos
            self.diagnostics_pub.publish(diagnostics)
            
            # Publicar status geral
            status_levels = ['OK', 'WARN', 'ERROR', 'STALE']
            status_msg = String()
            status_msg.data = status_levels[min(overall_level, 3)]
            self.system_status_pub.publish(status_msg)
            
            # Log detalhado (se habilitado)
            if self.detailed_logging and int(time.time()) % 60 == 0:  # A cada minuto
                self.get_logger().info(
                    f"Sistema: CPU {info['cpu_percent']:.1f}%, "
                    f"Temp {info['cpu_temp']:.1f}°C, "
                    f"Mem {info['memory_percent']:.1f}%, "
                    f"Disco {info['disk_percent']:.1f}%"
                )
            
        except Exception as e:
            self.get_logger().error(f'Erro no monitoramento do sistema: {e}')


def main(args=None):
    """Função principal."""
    rclpy.init(args=args)
    
    try:
        system_monitor = SystemMonitor()
        rclpy.spin(system_monitor)
    except KeyboardInterrupt:
        pass
    except Exception as e:
        print(f'Erro no system_monitor: {e}')
    finally:
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
