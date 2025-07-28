#!/bin/bash
# TortoiseBot Auto-Start Script
# Script para inicialização automática do TortoiseBot no boot

set -e

# Configurações
WORKSPACE_PATH="/home/pi/tortoisebot_ws"
LOG_DIR="/var/log/tortoisebot"
PID_FILE="/var/run/tortoisebot.pid"
USER="pi"

# Cores para output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m' # No Color

# Função de log
log() {
    echo -e "${BLUE}[$(date '+%Y-%m-%d %H:%M:%S')]${NC} $1"
}

error() {
    echo -e "${RED}[ERROR]${NC} $1" >&2
}

warn() {
    echo -e "${YELLOW}[WARN]${NC} $1"
}

success() {
    echo -e "${GREEN}[SUCCESS]${NC} $1"
}

# Função para verificar se o ROS2 está instalado
check_ros2() {
    if ! command -v ros2 &> /dev/null; then
        error "ROS2 não encontrado. Instale o ROS2 primeiro."
        exit 1
    fi
    
    # Verificar se o workspace existe
    if [ ! -d "$WORKSPACE_PATH" ]; then
        error "Workspace não encontrado em $WORKSPACE_PATH"
        exit 1
    fi
    
    log "ROS2 e workspace verificados"
}

# Função para criar diretórios necessários
setup_directories() {
    sudo mkdir -p "$LOG_DIR"
    sudo chown "$USER:$USER" "$LOG_DIR"
    log "Diretórios de log criados"
}

# Função para verificar hardware
check_hardware() {
    log "Verificando hardware..."
    
    # Verificar GPIOs
    if [ ! -d "/sys/class/gpio" ]; then
        warn "Sistema GPIO não detectado"
    else
        log "Sistema GPIO disponível"
    fi
    
    # Verificar dispositivos USB (câmera, LiDAR)
    if lsusb | grep -q "Lenovo"; then
        log "Câmera Lenovo detectada"
    else
        warn "Câmera Lenovo não detectada"
    fi
    
    # Verificar se o LiDAR está conectado
    if ls /dev/ttyUSB* 2>/dev/null | grep -q ttyUSB; then
        log "Dispositivos USB seriais detectados (possível LiDAR)"
    else
        warn "Nenhum dispositivo USB serial detectado"
    fi
    
    # Verificar espaço em disco
    DISK_USAGE=$(df / | awk 'NR==2 {print $5}' | sed 's/%//')
    if [ "$DISK_USAGE" -gt 80 ]; then
        warn "Espaço em disco baixo: ${DISK_USAGE}%"
    else
        log "Espaço em disco OK: ${DISK_USAGE}%"
    fi
}

# Função para configurar ambiente ROS2
setup_ros2_environment() {
    log "Configurando ambiente ROS2..."
    
    # Source do ROS2
    if [ -f "/opt/ros/humble/setup.bash" ]; then
        source /opt/ros/humble/setup.bash
    else
        error "ROS2 Humble não encontrado"
        exit 1
    fi
    
    # Source do workspace
    if [ -f "$WORKSPACE_PATH/install/setup.bash" ]; then
        source "$WORKSPACE_PATH/install/setup.bash"
    else
        error "Workspace não compilado. Execute 'colcon build' primeiro."
        exit 1
    fi
    
    # Configurar DDS
    export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
    export ROS_DOMAIN_ID=42
    
    log "Ambiente ROS2 configurado"
}

# Função para iniciar o TortoiseBot
start_tortoisebot() {
    log "Iniciando TortoiseBot..."
    
    # Verificar se já está rodando
    if [ -f "$PID_FILE" ] && kill -0 "$(cat $PID_FILE)" 2>/dev/null; then
        warn "TortoiseBot já está rodando (PID: $(cat $PID_FILE))"
        return 0
    fi
    
    # Configurar ambiente
    setup_ros2_environment
    
    # Iniciar o launch principal em background
    cd "$WORKSPACE_PATH"
    
    nohup ros2 launch tortoisebot_bringup bringup.launch.py \
        enable_lidar:=true \
        enable_camera:=true \
        enable_status_monitoring:=true \
        > "$LOG_DIR/tortoisebot.log" 2>&1 &
    
    TORTOISEBOT_PID=$!
    echo $TORTOISEBOT_PID > "$PID_FILE"
    
    # Aguardar alguns segundos para verificar se iniciou corretamente
    sleep 5
    
    if kill -0 $TORTOISEBOT_PID 2>/dev/null; then
        success "TortoiseBot iniciado com sucesso (PID: $TORTOISEBOT_PID)"
        
        # Verificar se os nós estão rodando
        sleep 10
        check_nodes
    else
        error "Falha ao iniciar TortoiseBot"
        cat "$LOG_DIR/tortoisebot.log"
        exit 1
    fi
}

# Função para verificar nós ROS2
check_nodes() {
    log "Verificando nós ROS2..."
    
    # Lista de nós esperados
    EXPECTED_NODES=(
        "/tortoisebot/differential_drive"
        "/tortoisebot/ultrasonic_sensor"
        "/tortoisebot/battery_monitor"
        "/tortoisebot/system_monitor"
    )
    
    for node in "${EXPECTED_NODES[@]}"; do
        if ros2 node list | grep -q "$node"; then
            log "✓ Nó ativo: $node"
        else
            warn "✗ Nó não encontrado: $node"
        fi
    done
}

# Função para parar o TortoiseBot
stop_tortoisebot() {
    log "Parando TortoiseBot..."
    
    if [ -f "$PID_FILE" ]; then
        PID=$(cat "$PID_FILE")
        if kill -0 "$PID" 2>/dev/null; then
            kill -TERM "$PID"
            
            # Aguardar término gracioso
            for i in {1..10}; do
                if ! kill -0 "$PID" 2>/dev/null; then
                    break
                fi
                sleep 1
            done
            
            # Forçar término se necessário
            if kill -0 "$PID" 2>/dev/null; then
                warn "Forçando término do processo..."
                kill -KILL "$PID"
            fi
            
            success "TortoiseBot parado"
        else
            warn "Processo não estava rodando"
        fi
        rm -f "$PID_FILE"
    else
        warn "Arquivo PID não encontrado"
    fi
}

# Função para mostrar status
show_status() {
    echo "=== Status do TortoiseBot ==="
    
    if [ -f "$PID_FILE" ] && kill -0 "$(cat $PID_FILE)" 2>/dev/null; then
        success "TortoiseBot está RODANDO (PID: $(cat $PID_FILE))"
        
        # Mostrar informações dos nós
        echo ""
        echo "Nós ROS2 ativos:"
        ros2 node list | grep tortoisebot || echo "Nenhum nó encontrado"
        
        # Mostrar informações de sistema
        echo ""
        echo "Informações do sistema:"
        echo "  CPU: $(cat /proc/loadavg | cut -d' ' -f1-3)"
        if [ -f "/sys/class/thermal/thermal_zone0/temp" ]; then
            TEMP=$(cat /sys/class/thermal/thermal_zone0/temp)
            TEMP_C=$((TEMP / 1000))
            echo "  Temperatura: ${TEMP_C}°C"
        fi
        echo "  Memória: $(free -h | grep 'Mem:' | awk '{print $3 "/" $2}')"
        echo "  Disco: $(df -h / | awk 'NR==2 {print $3 "/" $2 " (" $5 ")"}')"
        
    else
        error "TortoiseBot NÃO está rodando"
    fi
}

# Função para mostrar logs
show_logs() {
    if [ -f "$LOG_DIR/tortoisebot.log" ]; then
        echo "=== Últimas 50 linhas do log ==="
        tail -n 50 "$LOG_DIR/tortoisebot.log"
    else
        warn "Arquivo de log não encontrado"
    fi
}

# Função para instalar como serviço systemd
install_service() {
    log "Instalando serviço systemd..."
    
    # Criar arquivo de serviço
    cat > /tmp/tortoisebot.service << EOF
[Unit]
Description=TortoiseBot ROS2 Robot
After=network.target
Wants=network.target

[Service]
Type=forking
User=$USER
WorkingDirectory=$WORKSPACE_PATH
ExecStart=$PWD/scripts/autostart.sh start
ExecStop=$PWD/scripts/autostart.sh stop
ExecReload=$PWD/scripts/autostart.sh restart
PIDFile=$PID_FILE
Restart=on-failure
RestartSec=5
TimeoutStartSec=60
TimeoutStopSec=30

[Install]
WantedBy=multi-user.target
EOF

    # Instalar serviço
    sudo mv /tmp/tortoisebot.service /etc/systemd/system/
    sudo systemctl daemon-reload
    sudo systemctl enable tortoisebot.service
    
    success "Serviço systemd instalado e habilitado"
    log "Use 'sudo systemctl start tortoisebot' para iniciar"
    log "Use 'sudo systemctl status tortoisebot' para verificar status"
}

# Função para desinstalar serviço
uninstall_service() {
    log "Desinstalando serviço systemd..."
    
    sudo systemctl stop tortoisebot.service 2>/dev/null || true
    sudo systemctl disable tortoisebot.service 2>/dev/null || true
    sudo rm -f /etc/systemd/system/tortoisebot.service
    sudo systemctl daemon-reload
    
    success "Serviço systemd desinstalado"
}

# Função principal
main() {
    case "${1:-help}" in
        start)
            check_ros2
            setup_directories
            check_hardware
            start_tortoisebot
            ;;
        stop)
            stop_tortoisebot
            ;;
        restart)
            stop_tortoisebot
            sleep 2
            start_tortoisebot
            ;;
        status)
            show_status
            ;;
        logs)
            show_logs
            ;;
        install-service)
            install_service
            ;;
        uninstall-service)
            uninstall_service
            ;;
        help|*)
            echo "TortoiseBot Auto-Start Script"
            echo ""
            echo "Uso: $0 {start|stop|restart|status|logs|install-service|uninstall-service|help}"
            echo ""
            echo "Comandos:"
            echo "  start              - Iniciar TortoiseBot"
            echo "  stop               - Parar TortoiseBot"
            echo "  restart            - Reiniciar TortoiseBot"
            echo "  status             - Mostrar status atual"
            echo "  logs               - Mostrar logs recentes"
            echo "  install-service    - Instalar como serviço systemd"
            echo "  uninstall-service  - Desinstalar serviço systemd"
            echo "  help               - Mostrar esta ajuda"
            echo ""
            echo "Arquivos:"
            echo "  Log: $LOG_DIR/tortoisebot.log"
            echo "  PID: $PID_FILE"
            echo "  Workspace: $WORKSPACE_PATH"
            ;;
    esac
}

# Executar função principal
main "$@"
