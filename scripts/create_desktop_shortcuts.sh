#!/bin/bash
# TortoiseBot Desktop Shortcuts Creator
# Cria atalhos na área de trabalho para facilitar o uso do TortoiseBot

set -e

# Configurações
WORKSPACE_PATH="/home/pi/tortoisebot_ws"
DESKTOP_PATH="$HOME/Desktop"
SCRIPTS_PATH="$WORKSPACE_PATH/scripts"

# Cores para output
GREEN='\033[0;32m'
BLUE='\033[0;34m'
NC='\033[0m' # No Color

log() {
    echo -e "${BLUE}[INFO]${NC} $1"
}

success() {
    echo -e "${GREEN}[SUCCESS]${NC} $1"
}

# Verificar se o diretório Desktop existe
if [ ! -d "$DESKTOP_PATH" ]; then
    mkdir -p "$DESKTOP_PATH"
fi

log "Criando atalhos do TortoiseBot..."

# 1. Atalho para iniciar TortoiseBot
cat > "$DESKTOP_PATH/TortoiseBot Start.desktop" << EOF
[Desktop Entry]
Version=1.0
Type=Application
Name=TortoiseBot Start
Comment=Iniciar TortoiseBot ROS2 System
Exec=gnome-terminal -- bash -c "$SCRIPTS_PATH/autostart.sh start; read -p 'Pressione Enter para fechar...'"
Icon=robot
Terminal=false
Categories=Robotics;
EOF

# 2. Atalho para parar TortoiseBot
cat > "$DESKTOP_PATH/TortoiseBot Stop.desktop" << EOF
[Desktop Entry]
Version=1.0
Type=Application
Name=TortoiseBot Stop
Comment=Parar TortoiseBot ROS2 System
Exec=gnome-terminal -- bash -c "$SCRIPTS_PATH/autostart.sh stop; read -p 'Pressione Enter para fechar...'"
Icon=application-exit
Terminal=false
Categories=Robotics;
EOF

# 3. Atalho para status TortoiseBot
cat > "$DESKTOP_PATH/TortoiseBot Status.desktop" << EOF
[Desktop Entry]
Version=1.0
Type=Application
Name=TortoiseBot Status
Comment=Verificar status do TortoiseBot
Exec=gnome-terminal -- bash -c "$SCRIPTS_PATH/autostart.sh status; read -p 'Pressione Enter para fechar...'"
Icon=dialog-information
Terminal=false
Categories=Robotics;
EOF

# 4. Atalho para RViz
cat > "$DESKTOP_PATH/TortoiseBot RViz.desktop" << EOF
[Desktop Entry]
Version=1.0
Type=Application
Name=TortoiseBot RViz
Comment=Abrir RViz para visualização do TortoiseBot
Exec=gnome-terminal -- bash -c "cd $WORKSPACE_PATH && source install/setup.bash && ros2 launch tortoisebot_description display.launch.py"
Icon=applications-graphics
Terminal=false
Categories=Robotics;
EOF

# 5. Atalho para navegação autônoma
cat > "$DESKTOP_PATH/TortoiseBot Navigation.desktop" << EOF
[Desktop Entry]
Version=1.0
Type=Application
Name=TortoiseBot Navigation
Comment=Iniciar navegação autônoma do TortoiseBot
Exec=gnome-terminal -- bash -c "cd $WORKSPACE_PATH && source install/setup.bash && ros2 launch tortoisebot_bringup navigation.launch.py"
Icon=applications-games
Terminal=false
Categories=Robotics;
EOF

# 6. Atalho para teleop
cat > "$DESKTOP_PATH/TortoiseBot Teleop.desktop" << EOF
[Desktop Entry]
Version=1.0
Type=Application
Name=TortoiseBot Teleop
Comment=Controle manual do TortoiseBot via teclado
Exec=gnome-terminal -- bash -c "cd $WORKSPACE_PATH && source install/setup.bash && ros2 run teleop_twist_keyboard teleop_twist_keyboard"
Icon=input-keyboard
Terminal=true
Categories=Robotics;
EOF

# 7. Atalho para logs
cat > "$DESKTOP_PATH/TortoiseBot Logs.desktop" << EOF
[Desktop Entry]
Version=1.0
Type=Application
Name=TortoiseBot Logs
Comment=Visualizar logs do TortoiseBot
Exec=gnome-terminal -- bash -c "$SCRIPTS_PATH/autostart.sh logs; read -p 'Pressione Enter para fechar...'"
Icon=text-x-generic
Terminal=false
Categories=Robotics;
EOF

# 8. Atalho para terminal ROS2
cat > "$DESKTOP_PATH/TortoiseBot Terminal.desktop" << EOF
[Desktop Entry]
Version=1.0
Type=Application
Name=TortoiseBot Terminal
Comment=Terminal com ambiente ROS2 configurado
Exec=gnome-terminal -- bash -c "cd $WORKSPACE_PATH && source install/setup.bash && echo 'Ambiente ROS2 TortoiseBot carregado!' && bash"
Icon=terminal
Terminal=false
Categories=Robotics;
EOF

# Tornar atalhos executáveis
chmod +x "$DESKTOP_PATH"/*.desktop

# Permitir executar atalhos (para alguns sistemas)
for desktop_file in "$DESKTOP_PATH"/*.desktop; do
    if command -v gio &> /dev/null; then
        gio set "$desktop_file" metadata::trusted true
    fi
done

success "Atalhos criados no Desktop:"
ls -la "$DESKTOP_PATH"/*.desktop | grep TortoiseBot

log "Para usar os atalhos, clique duas vezes em cada um deles."
log "Se aparecer um aviso de segurança, clique em 'Confiar e Executar'."
