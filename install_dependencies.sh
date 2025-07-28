#!/bin/bash

# Script de instalação das dependências do TortoiseBot
# Execute este script no Raspberry Pi 4

echo "=== Instalação das Dependências do TortoiseBot ==="

# Atualizar sistema
echo "Atualizando sistema..."
sudo apt update && sudo apt upgrade -y

# Instalar ROS2 Humble (se não estiver instalado)
echo "Verificando instalação do ROS2 Humble..."
if ! command -v ros2 &> /dev/null; then
    echo "Instalando ROS2 Humble..."
    sudo apt install software-properties-common -y
    sudo add-apt-repository universe -y
    sudo apt update && sudo apt install curl -y
    sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
    echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null
    sudo apt update
    sudo apt install ros-humble-desktop -y
    sudo apt install python3-colcon-common-extensions -y
fi

# Instalar dependências do ROS2
echo "Instalando dependências do ROS2..."
sudo apt install -y \
    ros-humble-joint-state-publisher \
    ros-humble-robot-state-publisher \
    ros-humble-cartographer \
    ros-humble-cartographer-ros \
    ros-humble-teleop-twist-keyboard \
    ros-humble-teleop-twist-joy \
    ros-humble-xacro \
    ros-humble-nav2-bringup \
    ros-humble-nav2-map-server \
    ros-humble-nav2-lifecycle-manager \
    ros-humble-urdf \
    ros-humble-tf2-ros \
    ros-humble-tf2-tools

# Instalar dependências Python para Raspberry Pi
echo "Instalando dependências Python..."
sudo apt install -y \
    python3-pip \
    python3-rpi.gpio \
    python3-serial \
    python3-opencv \
    python3-numpy

# Instalar YDLiDAR SDK (se disponível)
echo "Configurando YDLiDAR..."
if [ ! -d "/tmp/YDLidar-SDK" ]; then
    cd /tmp
    git clone https://github.com/YDLIDAR/YDLidar-SDK.git
    cd YDLidar-SDK
    mkdir build
    cd build
    cmake ..
    make -j$(nproc)
    sudo make install
    sudo ldconfig
fi

# Configurar regras udev para LiDAR
echo "Configurando regras udev para LiDAR..."
sudo tee /etc/udev/rules.d/99-ydlidar.rules > /dev/null <<EOF
KERNEL=="ttyUSB*", ATTRS{idVendor}=="10c4", ATTRS{idProduct}=="ea60", MODE:="0666", GROUP:="dialout", SYMLINK+="ydlidar"
KERNEL=="ttyACM*", ATTRS{idVendor}=="0483", ATTRS{idProduct}=="5740", MODE:="0666", GROUP:="dialout", SYMLINK+="ydlidar"
KERNEL=="ttyUSB*", ATTRS{idVendor}=="067b", ATTRS{idProduct}=="2303", MODE:="0666", GROUP:="dialout", SYMLINK+="ydlidar"
EOF

sudo udevadm control --reload-rules
sudo udevadm trigger

# Adicionar usuário ao grupo dialout
sudo usermod -a -G dialout $USER

# Configurar autostart do ROS2
echo "Configurando variáveis de ambiente..."
if ! grep -q "source /opt/ros/humble/setup.bash" ~/.bashrc; then
    echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
fi

# Habilitar GPIO
echo "Habilitando interface GPIO..."
sudo raspi-config nonint do_spi 0
sudo raspi-config nonint do_i2c 0
sudo raspi-config nonint do_serial 0

# Instalar dependências para câmera
echo "Configurando câmera..."
sudo apt install -y \
    v4l-utils \
    ros-humble-v4l2-camera \
    ros-humble-camera-ros

echo "=== Instalação Concluída ==="
echo "Reinicie o sistema para aplicar todas as mudanças:"
echo "sudo reboot"
echo ""
echo "Após reiniciar, execute:"
echo "cd ~/ros2_ws"
echo "colcon build"
echo "source install/setup.bash"
