# Guia de Uso do TortoiseBot Personalizado

Este guia mostra como usar seu robô TortoiseBot personalizado com seus componentes específicos.

## Pré-requisitos

### Hardware Conectado
- ✅ Raspberry Pi 4 ligado e conectado à rede
- ✅ Driver Motor Ponte H conectado conforme GPIO especificado
- ✅ 4 motores DC conectados ao driver
- ✅ RPLiDAR A1M8 conectado via USB
- ✅ Sensor HC-SR04 conectado aos GPIOs 7 e 8
- ✅ Webcam Lenovo conectada via USB
- ✅ Power Bank e baterias carregadas

### Software Instalado
```bash
# Verificar ROS2
ros2 --version

# Verificar workspace compilado
ls ~/ros2_ws/install/tortoisebot_*

# Verificar dispositivos
lsusb  # Deve mostrar câmera e LiDAR
ls /dev/ttyUSB*  # Deve mostrar LiDAR
```

## Casos de Uso

### 1. Teste Básico - Verificar Hardware

Primeiro teste para verificar se tudo está funcionando:

```bash
# Terminal 1: Iniciar o robô básico
cd ~/ros2_ws
source install/setup.bash
ros2 launch tortoisebot_bringup bringup.launch.py use_sim_time:=False

# Terminal 2: Verificar tópicos
ros2 topic list
# Deve mostrar: /cmd_vel, /scan, /front_ultrasonic/scan, /camera/image_raw

# Terminal 3: Testar movimento simples
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist '{linear: {x: 0.1}, angular: {z: 0.0}}' --once

# Parar o robô
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist '{linear: {x: 0.0}, angular: {z: 0.0}}' --once
```

### 2. Teleoperação Manual

Controlar o robô com teclado:

```bash
# Terminal 1: Iniciar robô
ros2 launch tortoisebot_bringup bringup.launch.py use_sim_time:=False

# Terminal 2: Teleoperação
ros2 run teleop_twist_keyboard teleop_twist_keyboard

# Use as teclas:
# i - frente
# , - trás  
# j - girar esquerda
# l - girar direita
# k - parar
# q/z - aumentar/diminuir velocidade linear
# w/x - aumentar/diminuir velocidade angular
```

### 3. Mapeamento (SLAM) - Exploração

Criar um mapa do ambiente usando SLAM:

```bash
# Terminal 1: Iniciar SLAM
ros2 launch tortoisebot_bringup navigation.launch.py use_sim_time:=False exploration:=True

# Terminal 2: Teleoperação para explorar
ros2 run teleop_twist_keyboard teleop_twist_keyboard

# Mover o robô lentamente pelo ambiente
# O mapa será construído em tempo real no RViz

# Terminal 3: Salvar o mapa quando finalizar
ros2 run nav2_map_server map_saver_cli -f ~/meu_mapa
```

### 4. Navegação Autônoma

Usar um mapa salvo para navegação autônoma:

```bash
# Terminal 1: Iniciar navegação com mapa
ros2 launch tortoisebot_bringup navigation.launch.py \
  use_sim_time:=False \
  exploration:=False \
  map:=/home/pi/meu_mapa.yaml

# No RViz:
# 1. Use "2D Pose Estimate" para definir posição inicial
# 2. Use "Nav2 Goal" para definir destino
# 3. O robô navegará automaticamente
```

### 5. Simulação no Gazebo

Testar em simulação antes do robô real:

```bash
# Iniciar simulação
ros2 launch tortoisebot_bringup bringup.launch.py use_sim_time:=True

# Em outro terminal, testar navegação
ros2 launch tortoisebot_bringup navigation.launch.py \
  use_sim_time:=True \
  exploration:=True
```

## Monitoramento e Debug

### Verificar Status dos Sensores

```bash
# LiDAR
ros2 topic echo /scan --once

# Sensor ultrassônico
ros2 topic echo /front_ultrasonic/scan --once

# Câmera
ros2 run rqt_image_view rqt_image_view

# Estado dos motores
ros2 topic echo /left_motor_pwm
ros2 topic echo /right_motor_pwm
```

### Verificar Transformações

```bash
# Ver árvore de transformações
ros2 run tf2_tools view_frames

# Verificar transformação específica
ros2 run tf2_ros tf2_echo base_link laser_frame
```

### Logs e Debug

```bash
# Ver logs de um nó específico
ros2 node info /differential_drive_controller

# Verificar computação do robô
ros2 topic hz /scan  # Frequência do LiDAR
ros2 topic bw /cmd_vel  # Largura de banda dos comandos
```

## Solução de Problemas

### Robô não se move
```bash
# Verificar se o controlador está rodando
ros2 node list | grep differential

# Verificar GPIO
sudo dmesg | grep gpio

# Testar comando direto
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist '{linear: {x: 0.1}}' -r 10
```

### LiDAR não detectado
```bash
# Verificar USB
lsusb | grep -i lidar

# Verificar porta serial
ls -la /dev/ttyUSB*

# Verificar permissões
sudo chmod 666 /dev/ttyUSB0
```

### Sensor ultrassônico não funciona
```bash
# Verificar se o nó está rodando
ros2 node list | grep ultrasonic

# Testar GPIO manualmente
echo "7" > /sys/class/gpio/export
echo "out" > /sys/class/gpio/gpio7/direction
```

### Navegação não funciona
```bash
# Verificar Nav2
ros2 node list | grep nav2

# Verificar costmaps
ros2 topic echo /global_costmap/costmap_updates --once
ros2 topic echo /local_costmap/costmap_updates --once

# Verificar localização
ros2 topic echo /amcl_pose --once
```

## Personalizações Avançadas

### Ajustar Velocidades
Edite o arquivo de parâmetros Nav2:
```bash
nano ~/ros2_ws/src/tortoisebot_bringup/config/nav2_params.yaml

# Modificar:
max_vel_x: 0.4  # Velocidade linear máxima
max_vel_theta: 0.8  # Velocidade angular máxima
```

### Ajustar Sensibilidade dos Sensores
Para o ultrassônico:
```bash
nano ~/ros2_ws/src/tortoisebot_firmware/tortoisebot_firmware/ultrasonic_sensor.py

# Modificar:
self.max_range = 4.0  # Alcance máximo
self.min_range = 0.02  # Alcance mínimo
```

### Configurar Mapeamento
```bash
nano ~/ros2_ws/src/tortoisebot_bringup/config/cartographer.lua

# Ajustar para seu ambiente:
TRAJECTORY_BUILDER_2D.max_range = 8.  # Alcance máximo do LiDAR
TRAJECTORY_BUILDER_2D.min_range = 0.15  # Alcance mínimo
```

## Scripts Úteis

### Inicialização Automática
Criar script para iniciar automaticamente:
```bash
#!/bin/bash
# ~/start_tortoisebot.sh

source /opt/ros/humble/setup.bash
source ~/ros2_ws/install/setup.bash

ros2 launch tortoisebot_bringup bringup.launch.py use_sim_time:=False
```

### Backup de Mapas
```bash
#!/bin/bash
# ~/backup_maps.sh

DATE=$(date +%Y%m%d_%H%M%S)
mkdir -p ~/maps_backup
cp ~/meu_mapa.* ~/maps_backup/mapa_$DATE/
echo "Mapa salvo em ~/maps_backup/mapa_$DATE/"
```

## Segurança

### Limites de Segurança
- Velocidade máxima limitada a 0.4 m/s
- Sensor ultrassônico para detecção próxima
- Timeout automático se perder comunicação
- Botão de emergência (implementar físicamente)

### Monitoramento de Bateria
```bash
# Verificar tensão das baterias (implementar sensor)
# Adicionar alarme quando tensão baixa
```

Este guia deve cobrir a maioria dos casos de uso do seu TortoiseBot personalizado!
