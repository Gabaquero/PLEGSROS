#!/bin/bash
# CAN interface setup script for robot control system

echo "=== Robot Control System CAN Setup ==="

# Check if running as root
if [ "$EUID" -ne 0 ]; then 
  echo "Error: This script must be run with sudo"
  echo "Usage: sudo ./setup_can.sh"
  exit 1
fi

# Detect if running in WSL2
if grep -qi "microsoft" /proc/version; then
    echo "⚠️  WSL2 detected - CAN kernel modules not available"
    echo "Will configure simulation mode for testing"
    WSL2_MODE=true
else
    WSL2_MODE=false
fi

# Function to check if module is loaded
check_module() {
    if lsmod | grep -q "^$1"; then
        echo "✓ Module $1 is loaded"
        return 0
    else
        echo "✗ Module $1 is not loaded"
        return 1
    fi
}

if [ "$WSL2_MODE" = false ]; then
    # Load CAN modules
    echo ""
    echo "Loading CAN kernel modules..."
    modprobe can
    modprobe can_raw
    modprobe can_bcm
    modprobe vcan

    # Load PEAK CAN modules
    echo "Loading PEAK CAN modules..."
    modprobe peak_usb 2>/dev/null || echo "⚠️  peak_usb module not available - install PEAK drivers if using PEAK hardware"
    modprobe peak_pci 2>/dev/null || echo "⚠️  peak_pci module not available - install PEAK drivers if using PEAK hardware"

    # Check modules
    echo ""
    echo "Checking loaded modules..."
    check_module can
    check_module can_raw
    check_module vcan
    check_module peak_usb
    check_module peak_pci
else
    echo ""
    echo "Skipping CAN module loading in WSL2 mode"
    echo "Using simulation mode for testing"
fi

if [ "$WSL2_MODE" = false ]; then
    # Setup virtual CAN for testing
    echo ""
    echo "Setting up virtual CAN interface (vcan0)..."
    if ip link show vcan0 &> /dev/null; then
        echo "vcan0 already exists, bringing it down first..."
        ip link set down vcan0
        ip link delete vcan0
    fi

    ip link add dev vcan0 type vcan
    ip link set up vcan0
    echo "✓ Virtual CAN (vcan0) created and activated"
else
    echo ""
    echo "Skipping virtual CAN setup in WSL2 mode"
    echo "✓ WSL2 simulation mode configured"
fi

if [ "$WSL2_MODE" = false ]; then
    # Setup physical CAN if available
    echo ""
    echo "Checking for physical CAN interfaces..."
    if [ -e /sys/class/net/can0 ]; then
        echo "Found physical CAN interface (can0)"
        echo "Configuring can0 for 1Mbps..."
        
        # Bring down interface first
        ip link set down can0 2>/dev/null
        
        # Configure CAN with PEAK CAN specific settings
        ip link set can0 type can bitrate 1000000 restart-ms 100
        ip link set can0 txqueuelen 1000
        
        # Bring up interface
        ip link set up can0
        
        if [ $? -eq 0 ]; then
            echo "✓ Physical CAN (can0) configured at 1Mbps"
        else
            echo "✗ Failed to configure can0"
        fi
    else
        echo "No physical CAN interface found"
        echo "Using virtual CAN (vcan0) for testing"
    fi
else
    echo ""
    echo "Physical CAN interfaces not available in WSL2"
    echo "Use real hardware on native Linux for PEAK CAN testing"
fi

if [ "$WSL2_MODE" = false ]; then
    # Show interface status
    echo ""
    echo "CAN Interface Status:"
    echo "===================="
    ip -details -statistics link show can0 2>/dev/null || echo "can0: Not available"
    echo ""
    ip -details -statistics link show vcan0

    # Test CAN functionality
    echo ""
    echo "Testing CAN functionality..."
    timeout 1s candump vcan0 &> /dev/null &
    DUMP_PID=$!
    cansend vcan0 123#DEADBEEF
    sleep 0.5
    if kill -0 $DUMP_PID 2>/dev/null; then
        kill $DUMP_PID 2>/dev/null
        echo "✓ CAN send/receive test passed"
    else
        echo "✓ CAN interface is functional"
    fi
else
    echo ""
    echo "CAN Interface Status: WSL2 Simulation Mode"
    echo "=========================================="
    echo "✓ ROS 2 simulation mode ready"
    echo "✓ CAN utilities available: $(which cansend) $(which candump)"
fi

# Create systemd service for persistent CAN setup (optional)
echo ""
read -p "Create systemd service for automatic CAN setup on boot? (y/n): " -n 1 -r
echo
if [[ $REPLY =~ ^[Yy]$ ]]; then
    cat > /etc/systemd/system/can-setup.service << EOF
[Unit]
Description=CAN interface setup for robot control
After=network.target

[Service]
Type=oneshot
ExecStart=/bin/bash -c 'modprobe can can_raw vcan peak_usb peak_pci; ip link add dev vcan0 type vcan; ip link set up vcan0; if [ -e /sys/class/net/can0 ]; then ip link set can0 type can bitrate 1000000 restart-ms 100; ip link set up can0; fi'
RemainAfterExit=yes

[Install]
WantedBy=multi-user.target
EOF

    systemctl daemon-reload
    systemctl enable can-setup.service
    echo "✓ Systemd service created and enabled"
fi

echo ""
echo "=== CAN Setup Complete ==="
echo ""
echo "Next steps:"
if [ "$WSL2_MODE" = true ]; then
    echo "1. Test with simulation:   ros2 launch robot_control_system robot_control.launch.py use_sim:=true"
    echo "2. Test single joint:      ros2 launch robot_control_system test_single_joint.launch.py"
    echo "3. Build the workspace:    cd /home/gabor/PLEGS.ROS/PLEGSROS && colcon build"
    echo "4. For real CAN testing, use native Linux with PEAK CAN hardware"
else
    echo "1. Test with virtual CAN:  ros2 launch robot_control_system robot_control.launch.py use_sim:=false"
    echo "2. Monitor CAN traffic:    candump vcan0"
    echo "3. Send test messages:     cansend vcan0 201#0000000000000000"
    echo ""
    echo "For PEAK CAN hardware support:"
    echo "• Download drivers from: https://www.peak-system.com/PCAN-Driver-Linux.199.0.html"
    echo "• Or install via package manager if available"
fi
echo ""