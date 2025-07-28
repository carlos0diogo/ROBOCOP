# Configuração de Hardware - TortoiseBot Personalizado
# Este arquivo documenta as conexões GPIO para seu robô específico

## Componentes do Sistema

### Raspberry Pi 4
- Processador principal
- Executa ROS2 Humble
- Controla todos os sensores e atuadores

### Driver Motor Ponte H Duplo 3V-14V 5A (Motores Traseiros)
- **Localização**: Controle dos motores traseiros para tração diferencial
- **Alimentação**: Power Bank Samsung 10000mAh + Baterias AA

#### Conexões GPIO - Driver Traseiro:
| Função | Pino Driver | Pino Físico RPi | GPIO | Cabo |
|:-------|:------------|:----------------|:-----|:-----|
| Motor Esquerda I1 | I1 | 16 | GPIO 23 | - |
| Motor Esquerda I2 | I2 | 18 | GPIO 24 | - |
| Motor Direita I3 | I3 | 29 | GPIO 5 | - |
| Motor Direita I4 | I4 | 31 | GPIO 6 | - |
| Alimentação Lógica | VCC | 17 | 3.3V | - |
| Terra Comum | GND | 14 | GND | - |
| PWM Esquerdo | ENA | 32 | GPIO 12 | - |
| PWM Direito | ENB | 33 | GPIO 13 | - |

### Motores DC 3-6V com Caixa de Redução (4 unidades)
- **Tensão**: 3-6V
- **RPM**: ~60 RPM (estimativa com redução)
- **Configuração**: Tração nas 4 rodas (4WD)
- **Controle**: Apenas motores traseiros controlados eletronicamente

### RPLiDAR A1M8
- **Interface**: USB Serial (ttyUSB0)
- **Alcance**: 0.15m - 12m
- **Resolução Angular**: 1°
- **Frequência**: 10Hz
- **Montagem**: No topo do robô via suporte cilíndrico

### Sensor Ultrassônico HC-SR04
- **Alcance**: 2cm - 4m
- **Localização**: Frente do robô
- **Função**: Detecção de obstáculos próximos

#### Conexões GPIO - HC-SR04:
| Função | Pino HC-SR04 | Pino Físico RPi | GPIO |
|:-------|:-------------|:----------------|:-----|
| Trigger | TRIG | 26 | GPIO 7 |
| Echo | ECHO | 24 | GPIO 8 |
| VCC | VCC | 4 | 5V |
| GND | GND | 6 | GND |

### Webcam Lenovo 300 Full HD
- **Resolução**: 1920x1080 @ 30fps
- **Interface**: USB
- **Microfones**: 2 integrados
- **Função**: Visão computacional e navegação visual

### Sistema de Alimentação
1. **Power Bank Samsung 10000mAh**
   - Alimentação do Raspberry Pi 4
   - Alimentação da lógica dos drivers

2. **8x Baterias AA 2600mAh**
   - Alimentação dos motores DC
   - Configuração: 2 packs de 4 baterias (6V cada)

### Estrutura Robótica Acrílica
- **Material**: Acrílico com furos pré-definidos
- **Facilita**: Montagem de sensores e eletrônicos
- **Configuração**: Multi-níveis para organização

## Configuração de Software

### Mapeamento de Tópicos ROS2:
```
/cmd_vel                    # Comandos de velocidade
/scan                       # Dados do LiDAR
/front_ultrasonic/scan      # Dados do sensor ultrassônico
/camera/image_raw           # Imagem da webcam
/camera/camera_info         # Informações da câmera
/odom                       # Odometria
/tf                         # Transformações
/robot_description          # Descrição URDF
```

### Nós ROS2 Principais:
- `differential_drive_controller` - Controle dos motores
- `ydlidar_ros2_driver_node` - Driver do LiDAR
- `ultrasonic_sensor` - Sensor HC-SR04
- `robot_state_publisher` - Estado do robô
- `cartographer_node` - SLAM
- `nav2_*` - Navegação autônoma

## Comandos de Teste

### Teste dos Motores:
```bash
# Mover para frente
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist '{linear: {x: 0.2, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}'

# Girar no local
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist '{linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.5}}'

# Parar
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist '{linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}'
```

### Verificar Sensores:
```bash
# Listar tópicos ativos
ros2 topic list

# Ver dados do LiDAR
ros2 topic echo /scan

# Ver dados do ultrassônico
ros2 topic echo /front_ultrasonic/scan

# Ver imagem da câmera
ros2 run rqt_image_view rqt_image_view
```

## Segurança e Manutenção

### Verificações Antes do Uso:
1. **Alimentação**: Verificar carga da power bank e baterias AA
2. **Conexões**: Confirmar todas as conexões GPIO
3. **LiDAR**: Verificar se está conectado (lsusb)
4. **Câmera**: Testar com v4l2-ctl --list-devices

### Limitações de Segurança:
- Velocidade máxima limitada por software
- Sensor ultrassônico para detecção de obstáculos próximos
- Timeout automático se perder comunicação

### Manutenção Periódica:
- Limpeza das lentes do LiDAR e câmera
- Verificação das conexões GPIO
- Atualização do firmware dos sensores
- Calibração do sensor ultrassônico
