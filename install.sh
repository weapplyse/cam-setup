#!/usr/bin/env bash
#
# install.sh - Install camera streaming scripts for Jetson Orin Nano
#
# Usage:
#   sudo ./install.sh --dual     # Install dual camera setup (default)
#   sudo ./install.sh --single   # Install single camera setup
#   sudo ./install.sh --check    # Check installation status
#   sudo ./install.sh --help     # Show help
#

set -e

# Colors
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m'

# Script directory
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

# Default user
INSTALL_USER="${SUDO_USER:-aspace}"
INSTALL_HOME="/home/${INSTALL_USER}"

# -----------------------------
# Helper functions
# -----------------------------
log_info() { echo -e "${BLUE}[INFO]${NC} $*"; }
log_success() { echo -e "${GREEN}[OK]${NC} $*"; }
log_warn() { echo -e "${YELLOW}[WARN]${NC} $*"; }
log_error() { echo -e "${RED}[ERROR]${NC} $*"; }

check_root() {
    if [[ $EUID -ne 0 ]]; then
        log_error "This script must be run as root (use sudo)"
        exit 1
    fi
}

show_help() {
    cat << EOF
Camera Setup Installer for Jetson Orin Nano

Usage: sudo $(basename "$0") [OPTIONS]

Options:
  --dual        Install dual camera setup (side-by-side streaming)
  --single      Install single camera setup
  --check       Check current installation status
  --uninstall   Remove installed services and scripts
  --help        Show this help message

This script will:
  1. Copy camera scripts to ${INSTALL_HOME}/
  2. Copy focus control script
  3. Create example config file
  4. Install and enable systemd service
  5. Start the camera stream

Examples:
  sudo $(basename "$0") --dual      # Dual camera (CAM0 + CAM1 side-by-side)
  sudo $(basename "$0") --single    # Single camera (CAM0 or CAM1)

EOF
    exit 0
}

# -----------------------------
# Check functions
# -----------------------------
check_installation() {
    echo ""
    echo "=========================================="
    echo "  Camera Setup Installation Status"
    echo "=========================================="
    echo ""
    
    # Check scripts
    log_info "Checking installed scripts..."
    [[ -f "${INSTALL_HOME}/camera.sh" ]] && log_success "camera.sh installed" || log_warn "camera.sh not found"
    [[ -f "${INSTALL_HOME}/camera-dual.sh" ]] && log_success "camera-dual.sh installed" || log_warn "camera-dual.sh not found"
    [[ -f "${INSTALL_HOME}/set-camera-focus.py" ]] && log_success "set-camera-focus.py installed" || log_warn "set-camera-focus.py not found"
    [[ -f "${INSTALL_HOME}/camera-config.sh" ]] && log_success "camera-config.sh exists" || log_warn "camera-config.sh not found"
    
    echo ""
    log_info "Checking systemd services..."
    
    # Check dual service
    if systemctl is-enabled camera-dual.service &>/dev/null; then
        log_success "camera-dual.service enabled"
        if systemctl is-active camera-dual.service &>/dev/null; then
            log_success "camera-dual.service running"
        else
            log_warn "camera-dual.service not running"
        fi
    else
        log_info "camera-dual.service not enabled"
    fi
    
    # Check single service
    if systemctl is-enabled camera-stream.service &>/dev/null; then
        log_success "camera-stream.service enabled"
        if systemctl is-active camera-stream.service &>/dev/null; then
            log_success "camera-stream.service running"
        else
            log_warn "camera-stream.service not running"
        fi
    else
        log_info "camera-stream.service not enabled"
    fi
    
    echo ""
    log_info "Checking stream..."
    if ss -tlnp 2>/dev/null | grep -q ":5000"; then
        log_success "Stream listening on port 5000"
    else
        log_warn "No stream on port 5000"
    fi
    
    echo ""
}

# -----------------------------
# Installation functions
# -----------------------------
install_scripts() {
    log_info "Installing camera scripts to ${INSTALL_HOME}..."
    
    # Copy main scripts
    cp "${SCRIPT_DIR}/camera.sh" "${INSTALL_HOME}/"
    cp "${SCRIPT_DIR}/camera-dual.sh" "${INSTALL_HOME}/"
    cp "${SCRIPT_DIR}/set-camera-focus.py" "${INSTALL_HOME}/"
    
    # Make executable
    chmod +x "${INSTALL_HOME}/camera.sh"
    chmod +x "${INSTALL_HOME}/camera-dual.sh"
    chmod +x "${INSTALL_HOME}/set-camera-focus.py"
    
    # Set ownership
    chown "${INSTALL_USER}:${INSTALL_USER}" "${INSTALL_HOME}/camera.sh"
    chown "${INSTALL_USER}:${INSTALL_USER}" "${INSTALL_HOME}/camera-dual.sh"
    chown "${INSTALL_USER}:${INSTALL_USER}" "${INSTALL_HOME}/set-camera-focus.py"
    
    log_success "Scripts installed"
    
    # Create config if it doesn't exist
    if [[ ! -f "${INSTALL_HOME}/camera-config.sh" ]]; then
        log_info "Creating default camera-config.sh..."
        cp "${SCRIPT_DIR}/camera-config.sh.example" "${INSTALL_HOME}/camera-config.sh"
        chown "${INSTALL_USER}:${INSTALL_USER}" "${INSTALL_HOME}/camera-config.sh"
        log_success "Config file created"
    else
        log_info "camera-config.sh already exists, keeping existing config"
    fi
}

install_dual_service() {
    log_info "Installing dual camera service..."
    
    # Stop existing services
    systemctl stop camera-dual.service 2>/dev/null || true
    systemctl stop camera-stream.service 2>/dev/null || true
    systemctl disable camera-stream.service 2>/dev/null || true
    
    # Create service file
    cat > /etc/systemd/system/camera-dual.service << EOF
[Unit]
Description=Dual Camera Stream Service (camera-dual.sh)
After=network.target nvargus-daemon.service
Wants=nvargus-daemon.service

[Service]
Type=simple
User=${INSTALL_USER}
Group=video
WorkingDirectory=${INSTALL_HOME}

# Give camera hardware time to initialize
ExecStartPre=/bin/sleep 3

# Start the dual camera stream
ExecStart=${INSTALL_HOME}/camera-dual.sh --mode 2

Restart=always
RestartSec=5
StartLimitInterval=0
StandardOutput=journal
StandardError=journal

# Environment
Environment="PORT=5000"
Environment="SENSOR_MODE=2"
Environment="FOCUS=250"

[Install]
WantedBy=multi-user.target
EOF

    # Reload and enable
    systemctl daemon-reload
    systemctl enable camera-dual.service
    
    log_success "camera-dual.service installed and enabled"
}

install_single_service() {
    log_info "Installing single camera service..."
    
    # Stop existing services
    systemctl stop camera-dual.service 2>/dev/null || true
    systemctl stop camera-stream.service 2>/dev/null || true
    systemctl disable camera-dual.service 2>/dev/null || true
    
    # Create service file
    cat > /etc/systemd/system/camera-stream.service << EOF
[Unit]
Description=Camera Stream Service (camera.sh)
After=network.target nvargus-daemon.service
Wants=nvargus-daemon.service

[Service]
Type=simple
User=${INSTALL_USER}
Group=video
WorkingDirectory=${INSTALL_HOME}

# Give camera hardware time to initialize
ExecStartPre=/bin/sleep 3

# Start the camera stream
ExecStart=${INSTALL_HOME}/camera.sh

Restart=always
RestartSec=5
StartLimitInterval=0
StandardOutput=journal
StandardError=journal

# Environment
Environment="PORT=5000"
Environment="SENSOR_ID=0"
Environment="FOCUS=250"

[Install]
WantedBy=multi-user.target
EOF

    # Reload and enable
    systemctl daemon-reload
    systemctl enable camera-stream.service
    
    log_success "camera-stream.service installed and enabled"
}

start_service() {
    local service="$1"
    
    log_info "Starting ${service}..."
    
    # Remove lock files
    rm -f /var/lock/camera-dual.lock
    rm -f /var/lock/camera-stream.lock
    
    # Kill any existing camera processes
    pkill -f "camera-dual.sh" 2>/dev/null || true
    pkill -f "camera.sh" 2>/dev/null || true
    sleep 2
    
    # Start service
    if systemctl start "${service}"; then
        sleep 5
        if systemctl is-active "${service}" &>/dev/null; then
            log_success "${service} started successfully"
            return 0
        fi
    fi
    
    log_error "Failed to start ${service}"
    log_info "Check logs with: journalctl -u ${service} -f"
    return 1
}

uninstall() {
    log_info "Uninstalling camera services..."
    
    # Stop and disable services
    systemctl stop camera-dual.service 2>/dev/null || true
    systemctl stop camera-stream.service 2>/dev/null || true
    systemctl disable camera-dual.service 2>/dev/null || true
    systemctl disable camera-stream.service 2>/dev/null || true
    
    # Remove service files
    rm -f /etc/systemd/system/camera-dual.service
    rm -f /etc/systemd/system/camera-stream.service
    
    systemctl daemon-reload
    
    log_success "Services removed"
    log_info "Scripts in ${INSTALL_HOME}/ were kept. Remove manually if needed."
}

# -----------------------------
# Main
# -----------------------------
main() {
    local mode=""
    local check_only=false
    local do_uninstall=false
    
    # Parse arguments
    while [[ $# -gt 0 ]]; do
        case "$1" in
            --dual)
                mode="dual"
                shift
                ;;
            --single)
                mode="single"
                shift
                ;;
            --check)
                check_only=true
                shift
                ;;
            --uninstall)
                do_uninstall=true
                shift
                ;;
            --help|-h)
                show_help
                ;;
            *)
                log_error "Unknown option: $1"
                show_help
                ;;
        esac
    done
    
    # Check only mode
    if $check_only; then
        check_installation
        exit 0
    fi
    
    # Uninstall mode
    if $do_uninstall; then
        check_root
        uninstall
        exit 0
    fi
    
    # Default to dual if no mode specified
    if [[ -z "$mode" ]]; then
        mode="dual"
    fi
    
    check_root
    
    echo ""
    echo "=========================================="
    echo "  Camera Setup Installer"
    echo "  Mode: ${mode}"
    echo "=========================================="
    echo ""
    
    # Install scripts
    install_scripts
    
    # Install service based on mode
    if [[ "$mode" == "dual" ]]; then
        install_dual_service
        start_service "camera-dual.service"
    else
        install_single_service
        start_service "camera-stream.service"
    fi
    
    echo ""
    echo "=========================================="
    echo "  Installation Complete!"
    echo "=========================================="
    echo ""
    
    if [[ "$mode" == "dual" ]]; then
        log_success "Dual camera stream installed"
        echo ""
        echo "Service commands:"
        echo "  sudo systemctl status camera-dual.service"
        echo "  sudo systemctl restart camera-dual.service"
        echo "  sudo journalctl -u camera-dual.service -f"
    else
        log_success "Single camera stream installed"
        echo ""
        echo "Service commands:"
        echo "  sudo systemctl status camera-stream.service"
        echo "  sudo systemctl restart camera-stream.service"
        echo "  sudo journalctl -u camera-stream.service -f"
    fi
    
    echo ""
    echo "View stream:"
    echo "  ffplay tcp://$(hostname -I | awk '{print $1}'):5000"
    echo ""
    echo "Edit config:"
    echo "  nano ${INSTALL_HOME}/camera-config.sh"
    echo ""
}

main "$@"
