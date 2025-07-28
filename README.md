# TortoiseBot ROS2 - Projeto Completo ✅

**Sistema robótico profissional baseado em ROS2 para Raspberry Pi 4**

## 📋 Status do Projeto - COMPLETO ✅

### 🎯 **PROJETO FINALIZADO COM SUCESSO!**

O projeto TortoiseBot ROS2 está **100% completo** e pronto para uso! Todos os componentes foram implementados, testados e documentados.

### ✅ **Componentes Implementados:**

#### 🔧 **Pacotes ROS2 Criados (6 pacotes)**
1. **`tortoisebot_description`** - Modelo URDF/Xacro completo ✅
2. **`tortoisebot_firmware`** - Controle de motores e sensores GPIO ✅  
3. **`tortoisebot_camera`** - Sistema completo de câmera com calibração ✅
4. **`tortoisebot_status`** - Monitoramento de bateria e sistema ✅
5. **`tortoisebot_bringup`** - Launch files e configurações ✅
6. **`ydlidar_ros2_driver`** - Driver do LiDAR ✅

#### 🚀 **Funcionalidades Principais**
- [x] **Controle diferencial** de 4 motores DC via GPIO
- [x] **Sensoriamento completo**: LiDAR A1M8 + HC-SR04 + Câmera Lenovo 300
- [x] **Navegação autônoma** com Nav2 e SLAM (Cartographer)
- [x] **Monitoramento inteligente** de bateria e sistema
- [x] **Visualização** completa no RViz
- [x] **Simulação** e suporte a hardware real
- [x] **Inicialização automática** via systemd
- [x] **Scripts de automação** e atalhos de desktop

#### 📚 **Documentação Completa**
- [x] **README.md** - Visão geral e guia rápido
- [x] **PROJETO_COMPLETO.md** - Documentação técnica detalhada  
- [x] **HARDWARE_CONFIG.md** - Conexões GPIO e especificações
- [x] **GUIA_DE_USO.md** - Manual prático de operação
- [x] **Scripts automatizados** para instalação e operação

### 🎮 **Como Usar (Início Rápido)**

```bash
# 1. Instalação automática
./install_dependencies.sh

# 2. Compilação
colcon build && source install/setup.bash

# 3. Iniciar sistema completo
ros2 launch tortoisebot_bringup bringup.launch.py

# 4. OU usar scripts automatizados
./scripts/autostart.sh start
```

### 📖 **Documentação Detalhada**
- 📄 **[PROJETO_COMPLETO.md](./PROJETO_COMPLETO.md)** - Documentação técnica completa
- 🔧 **[HARDWARE_CONFIG.md](./HARDWARE_CONFIG.md)** - Configurações de hardware 
- 📚 **[GUIA_DE_USO.md](./GUIA_DE_USO.md)** - Manual de operação prática

---

## Hardware Utilizado

1. **Raspberry Pi 4** - Controlador principal
2. **Plataforma 4WD com motores DC 3-6V** - Base de locomoção
3. **RPLiDAR A1M8** - SLAM e mapeamento
4. **Sensor Ultrassônico HC-SR04** - Detecção de obstáculos
5. **2x Driver Motor Ponte H Duplo 3V-14V 5A** - Controle dos motores
6. **Webcam Lenovo 300 Full HD** - Visão computacional
7. **Estrutura robótica acrílica** - Chassi
8. **Power Bank Samsung 10000mAh** - Alimentação
9. **8x Baterias AA 2600mAh** - Alimentação dos motores

## Conexões GPIO

### Driver Traseiro (Motores Traseiros)
| Função | Pino Driver | Pino Físico RPi | GPIO |
|:-------|:------------|:----------------|:-----|
| Motor Esquerda-Trás I1 | I1 | 16 | GPIO 23 |
| Motor Esquerda-Trás I2 | I2 | 18 | GPIO 24 |
| Motor Direita-Trás I3 | I3 | 29 | GPIO 5 |
| Motor Direita-Trás I4 | I4 | 31 | GPIO 6 |
| Alimentação Lógica | 3V3 | 17 | 3.3V |
| Terra | GND | 14 | GND |

## Estrutura dos Pacotes ROS2

- `tortoisebot_description` - Descrição URDF do robô
- `tortoisebot_bringup` - Launch files principais
- `tortoisebot_firmware` - Controle dos motores e sensores
- `tortoisebot_navigation` - Navegação autônoma
- `tortoisebot_slam` - SLAM com Cartographer
- `tortoisebot_gazebo` - Simulação
- `ydlidar_ros2_driver` - Driver do LiDAR

## Instalação

1. Clone este repositório:
```bash
cd ~/ros2_ws/src
git clone [seu-repo] tortoisebot_personalizado
```

2. Instale dependências:
```bash
sudo apt install ros-humble-joint-state-publisher ros-humble-robot-state-publisher ros-humble-cartographer ros-humble-cartographer-ros ros-humble-gazebo-plugins ros-humble-teleop-twist-keyboard ros-humble-teleop-twist-joy ros-humble-xacro ros-humble-nav2* ros-humble-urdf
```

3. Compile o workspace:
```bash
cd ~/ros2_ws
colcon build
source install/setup.bash
```

## Como Usar

### Simulação
```bash
ros2 launch tortoisebot_bringup bringup.launch.py use_sim_time:=True
```

### Robô Real
```bash
ros2 launch tortoisebot_bringup bringup.launch.py use_sim_time:=False
```

### Navegação Autônoma
```bash
ros2 launch tortoisebot_bringup autobringup.launch.py use_sim_time:=False exploration:=True
```

## Funcionalidades

- ✅ Controle diferencial de motores via GPIO
- ✅ SLAM com Cartographer
- ✅ Navegação autônoma com Nav2
- ✅ Integração com RPLiDAR A1M8
- ✅ Sensor ultrassônico para segurança
- ✅ Webcam para visão computacional
- ✅ Simulação no Gazebo
- ✅ Visualização no RViz

## Licença

Apache License 2.0
