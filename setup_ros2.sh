#!/bin/bash

# Script para configurar ROS2 Humble quando não há setup.bash
echo "Configurando ROS2 Humble..."

# Tentar diferentes localizações do ROS2
if [ -f "/opt/ros/humble/setup.bash" ]; then
    source /opt/ros/humble/setup.bash
    echo "ROS2 Humble carregado de /opt/ros/humble/"
elif [ -f "$HOME/ros2_humble/install/setup.bash" ]; then
    source $HOME/ros2_humble/install/setup.bash
    echo "ROS2 Humble carregado de $HOME/ros2_humble/"
else
    echo "ERRO: ROS2 Humble não encontrado!"
    echo "Instale o ROS2 Humble primeiro ou compile do código fonte."
    exit 1
fi

# Configurar variáveis de ambiente
export ROS_DOMAIN_ID=0
export ROS_LOCALHOST_ONLY=0

echo "ROS2 configurado com sucesso!"
echo "ROS_DISTRO: $ROS_DISTRO"
echo "ROS_VERSION: $ROS_VERSION"
