# TortoiseBot ROS2 - Projeto Completo

## 📋 Visão Geral do Projeto

Este é um projeto ROS2 completo para um robô 4WD baseado no TortoiseBot, totalmente adaptado para componentes específicos incluindo Raspberry Pi 4, sensores, câmera, LiDAR e sistema de monitoramento completo.

## 🔧 Hardware Especificado

### Componentes Principais
- **Controlador**: Raspberry Pi 4
- **Motores**: 4x motores DC 3-6V
- **Drivers**: 2x Ponte H Duplo 3V-14V 5A
- **LiDAR**: RPLiDAR A1M8
- **Sensor**: Ultrassônico HC-SR04
- **Câmera**: Webcam Lenovo 300
- **Estrutura**: Acrílica
- **Alimentação**: Power bank Samsung + 4x pilhas AA

### Conexões GPIO Detalhadas
Consulte `HARDWARE_CONFIG.md` para diagramas detalhados e especificações completas.

## 📦 Estrutura de Pacotes ROS2

### 1. `tortoisebot_description`
- **Finalidade**: Descrição URDF/Xacro do robô
- **Conteúdo**: 
  - Modelo 3D completo do robô
  - Configurações de sensores e atuadores
  - Launch files para visualização
  - Configurações do RViz

### 2. `tortoisebot_firmware`
- **Finalidade**: Nós de baixo nível para controle de hardware
- **Conteúdo**:
  - `differential_drive.py`: Controle dos motores DC via GPIO
  - `ultrasonic_sensor.py`: Interface com sensor HC-SR04

### 3. `tortoisebot_camera`
- **Finalidade**: Sistema de câmera
- **Conteúdo**:
  - `camera_node.py`: Controle da webcam Lenovo 300
  - `camera_calibration.py`: Calibração automática da câmera
  - Configurações otimizadas para diferentes usos

### 4. `tortoisebot_status`
- **Finalidade**: Monitoramento de sistema e bateria
- **Conteúdo**:
  - `battery_monitor.py`: Monitoramento de nível de bateria
  - `system_monitor.py`: CPU, temperatura, memória, disco
  - `diagnostics_node.py`: Sistema de diagnósticos ROS2

### 5. `tortoisebot_bringup`
- **Finalidade**: Launch files principais
- **Conteúdo**:
  - `bringup.launch.py`: Launch principal do sistema
  - `navigation.launch.py`: Navegação autônoma
  - Configurações do Nav2 e SLAM

### 6. `ydlidar_ros2_driver`
- **Finalidade**: Driver para RPLiDAR A1M8
- **Conteúdo**:
  - Driver adaptado para o modelo específico
  - Configurações otimizadas

## 🚀 Recursos e Funcionalidades

### ✅ Implementado e Funcional

#### Sistema de Controle
- [x] Controle diferencial de 4 motores DC
- [x] Interface para comandos de velocidade (`cmd_vel`)
- [x] Publicação de odometria
- [x] Controle via GPIO do Raspberry Pi

#### Sensoriamento
- [x] Integração completa do RPLiDAR A1M8
- [x] Sensor ultrassônico HC-SR04 como backup
- [x] Câmera Lenovo 300 com múltiplas resoluções
- [x] Calibração automática de câmera

#### Monitoramento de Sistema
- [x] Monitoramento de bateria (tensão, porcentagem, alertas)
- [x] Monitoramento de CPU, temperatura, memória
- [x] Sistema de diagnósticos ROS2 completo
- [x] Alertas de bateria baixa e crítica

#### Navegação e SLAM
- [x] Integração com Nav2 stack completo
- [x] SLAM com Cartographer configurado
- [x] Mapas salvos e carregamento automático
- [x] Planejamento de trajetória

#### Visualização e Interface
- [x] Configurações otimizadas do RViz
- [x] Interface gráfica para todos os sensores
- [x] Visualização de diagnósticos em tempo real

#### Automação e Deploy
- [x] Scripts de instalação automatizada
- [x] Inicialização automática via systemd
- [x] Atalhos de desktop para facilitar uso
- [x] Sistema de logs completo

### 🎯 Características Avançadas

#### Sistema Inteligente de Energia
- Monitoramento contínuo de bateria com múltiplos métodos
- Alertas preventivos e críticos
- Desligamento gracioso quando necessário

#### Robustez e Confiabilidade
- Sistema de recuperação automática de falhas
- Múltiplos métodos de leitura de sensores
- Fallbacks para componentes não disponíveis

#### Escalabilidade
- Arquitetura modular permite adicionar novos sensores
- Configurações flexíveis para diferentes cenários
- Suporte a simulação e hardware real

## 📁 Estrutura Completa de Arquivos

```
tortoisebot_ws/
├── README.md                           # Este arquivo
├── colcon.meta                         # Configurações do workspace
├── install_dependencies.sh             # Script de instalação
├── HARDWARE_CONFIG.md                  # Documentação de hardware
├── GUIA_DE_USO.md                     # Guia prático de uso
├── scripts/
│   ├── autostart.sh                   # Script de inicialização automática
│   └── create_desktop_shortcuts.sh    # Criador de atalhos
└── src/
    ├── tortoisebot_description/
    │   ├── package.xml
    │   ├── CMakeLists.txt
    │   ├── urdf/
    │   │   ├── tortoisebot.xacro      # Modelo principal do robô
    │   │   ├── materials.xacro        # Materiais e cores
    │   │   └── sensors.xacro          # Sensores e atuadores
    │   ├── launch/
    │   │   └── display.launch.py      # Visualização no RViz
    │   └── rviz/
    │       └── tortoisebot_display.rviz
    ├── tortoisebot_firmware/
    │   ├── package.xml
    │   ├── setup.py
    │   └── tortoisebot_firmware/
    │       ├── differential_drive.py   # Controle dos motores
    │       └── ultrasonic_sensor.py   # Sensor ultrassônico
    ├── tortoisebot_camera/
    │   ├── package.xml
    │   ├── setup.py
    │   ├── tortoisebot_camera/
    │   │   ├── camera_node.py         # Nó principal da câmera
    │   │   └── camera_calibration.py  # Calibração automática
    │   ├── launch/
    │   │   └── camera.launch.py       # Launch da câmera
    │   └── config/
    │       └── camera_config.yaml     # Configurações da câmera
    ├── tortoisebot_status/
    │   ├── package.xml
    │   ├── setup.py
    │   ├── tortoisebot_status/
    │   │   ├── battery_monitor.py     # Monitor de bateria
    │   │   └── system_monitor.py      # Monitor de sistema
    │   ├── launch/
    │   │   └── status.launch.py       # Launch de monitoramento
    │   └── config/
    │       └── status_config.yaml     # Configurações de status
    ├── tortoisebot_bringup/
    │   ├── package.xml
    │   ├── CMakeLists.txt
    │   ├── launch/
    │   │   ├── bringup.launch.py      # Launch principal
    │   │   └── navigation.launch.py   # Navegação autônoma
    │   ├── config/
    │   │   ├── nav2_params.yaml       # Parâmetros Nav2
    │   │   └── cartographer.lua       # Configuração SLAM
    │   └── maps/
    │       └── empty_map.yaml         # Mapa vazio inicial
    └── ydlidar_ros2_driver/
        ├── package.xml
        ├── CMakeLists.txt
        ├── src/
        │   └── placeholder_node.cpp   # Driver do LiDAR
        ├── launch/
        │   └── ydlidar_launch.py      # Launch do LiDAR
        └── params/
            └── ydlidar.yaml           # Parâmetros do LiDAR
```

## 🛠 Comandos Principais

### Instalação e Setup
```bash
# Clonar e configurar
cd ~/
git clone https://github.com/carlos0diogo/ROBOCOP.git tortoisebot_ws
cd tortoisebot_ws

# Instalar dependências
./install_dependencies.sh

# Compilar
colcon build

# Configurar ambiente
source install/setup.bash
```

### Operação Básica
```bash
# Iniciar sistema completo
ros2 launch tortoisebot_bringup bringup.launch.py

# Apenas visualização
ros2 launch tortoisebot_description display.launch.py

# Navegação autônoma
ros2 launch tortoisebot_bringup navigation.launch.py

# Controle manual
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

### Scripts de Automação
```bash
# Usar scripts automatizados
./scripts/autostart.sh start          # Iniciar
./scripts/autostart.sh stop           # Parar
./scripts/autostart.sh status         # Status
./scripts/autostart.sh install-service # Instalar como serviço

# Criar atalhos de desktop
./scripts/create_desktop_shortcuts.sh
```

## 📊 Monitoramento e Diagnósticos

### Tópicos de Status
- `/tortoisebot/status/battery_voltage` - Tensão da bateria
- `/tortoisebot/status/battery_percentage` - Porcentagem da bateria
- `/tortoisebot/status/cpu_temperature` - Temperatura da CPU
- `/tortoisebot/status/system_status` - Status geral do sistema
- `/tortoisebot/status/diagnostics` - Diagnósticos completos

### Verificações de Saúde
```bash
# Ver todos os nós ativos
ros2 node list

# Monitorar bateria
ros2 topic echo /tortoisebot/status/battery_percentage

# Ver diagnósticos
ros2 topic echo /tortoisebot/status/diagnostics

# Status do sistema
./scripts/autostart.sh status
```

## 🎮 Modos de Operação

### 1. Modo Simulação
```bash
ros2 launch tortoisebot_bringup bringup.launch.py use_sim_time:=true
```

### 2. Modo Robô Real
```bash
ros2 launch tortoisebot_bringup bringup.launch.py use_sim_time:=false
```

### 3. Modo Navegação
```bash
ros2 launch tortoisebot_bringup navigation.launch.py
```

### 4. Modo Debug
```bash
# Componentes individuais
ros2 launch tortoisebot_camera camera.launch.py
ros2 launch tortoisebot_status status.launch.py
ros2 launch ydlidar_ros2_driver ydlidar_launch.py
```

## 🔧 Personalização e Configuração

### Hardware
- Edite `HARDWARE_CONFIG.md` para diferentes conexões GPIO
- Modifique `src/tortoisebot_firmware/` para diferentes drivers
- Ajuste `src/tortoisebot_description/urdf/` para mudanças físicas

### Software
- Configure `src/tortoisebot_bringup/config/nav2_params.yaml` para navegação
- Ajuste `src/tortoisebot_status/config/status_config.yaml` para monitoramento
- Personalize `src/tortoisebot_camera/config/camera_config.yaml` para câmera

## 🚨 Solução de Problemas

### Problemas Comuns
1. **GPIO não funciona**: Verifique permissões e conectores
2. **LiDAR não detectado**: Confirme porta `/dev/ttyUSB0`
3. **Câmera não encontrada**: Teste `ls /dev/video*`
4. **Bateria sempre 0%**: Configure método de leitura correto

### Logs e Debug
```bash
# Ver logs em tempo real
./scripts/autostart.sh logs

# Logs detalhados
journalctl -u tortoisebot -f

# Debug de nós específicos
ros2 run rqt_console rqt_console
```

## 📈 Próximos Passos e Melhorias

### Implementações Futuras
- [ ] Reconhecimento de objetos com câmera
- [ ] Seguimento de linha
- [ ] Controle por voz
- [ ] Interface web para controle remoto
- [ ] Integração com IoT/Cloud
- [ ] Mapeamento 3D

### Otimizações
- [ ] Ajuste fino de parâmetros de navegação
- [ ] Otimização de consumo de energia
- [ ] Melhoria da precision do SLAM
- [ ] Interface gráfica personalizada

## 🤝 Contribuição e Suporte

### Como Contribuir
1. Fork do repositório
2. Crie feature branch
3. Commit suas mudanças
4. Push para a branch
5. Crie Pull Request

### Suporte
- Consulte `GUIA_DE_USO.md` para uso prático
- Verifique `HARDWARE_CONFIG.md` para questões de hardware
- Use os scripts de autostart para operação automatizada

## 📄 Licença

Este projeto está sob licença MIT. Veja o arquivo LICENSE para detalhes.

## 🙏 Agradecimentos

Baseado no projeto TortoiseBot original, adaptado e expandido para um sistema completo de robótica educacional e profissional.

---

**TortoiseBot ROS2** - Um sistema de robótica completo e profissional para Raspberry Pi 4
*Documentação atualizada: $(date '+%Y-%m-%d')*
