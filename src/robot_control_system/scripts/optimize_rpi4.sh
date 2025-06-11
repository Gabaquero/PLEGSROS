#!/bin/bash
# Raspberry Pi 4 Real-Time Performance Optimization Script
# For use with real-time Ubuntu kernel

echo "=== Raspberry Pi 4 Real-Time Optimization ==="

# Check if running as root
if [ "$EUID" -ne 0 ]; then 
  echo "Error: This script must be run with sudo"
  echo "Usage: sudo ./optimize_rpi4.sh"
  exit 1
fi

# Check if running on RPi4
if ! grep -q "Raspberry Pi 4" /proc/device-tree/model 2>/dev/null; then
  echo "Warning: Not running on Raspberry Pi 4. Some optimizations may not apply."
fi

echo "Applying Raspberry Pi 4 specific optimizations..."

# 1. CPU Isolation and IRQ Affinity
echo "Setting up CPU isolation..."

# Isolate cores 2 and 3 for real-time tasks
if ! grep -q "isolcpus" /proc/cmdline; then
  echo "Adding CPU isolation to boot config..."
  echo "Please add 'isolcpus=2,3' to /boot/firmware/cmdline.txt and reboot"
else
  echo "✓ CPU isolation already configured"
fi

# Move IRQs away from real-time cores
echo "Configuring IRQ affinity..."
for irq in /proc/irq/*/smp_affinity; do
  if [ -w "$irq" ]; then
    echo "3" > "$irq" 2>/dev/null  # Bind to cores 0,1
  fi
done
echo "✓ IRQs moved to cores 0,1"

# 2. Memory Management
echo "Optimizing memory management..."

# Disable swap (critical for RT)
swapoff -a
echo "✓ Swap disabled"

# Set VM parameters for real-time
sysctl -w vm.swappiness=1
sysctl -w vm.dirty_ratio=5
sysctl -w vm.dirty_background_ratio=2
sysctl -w vm.dirty_expire_centisecs=500
sysctl -w vm.dirty_writeback_centisecs=100
echo "✓ VM parameters optimized"

# 3. CPU Governor and Frequency Scaling
echo "Setting CPU governor to performance..."
echo "performance" > /sys/devices/system/cpu/cpu0/cpufreq/scaling_governor
echo "performance" > /sys/devices/system/cpu/cpu1/cpufreq/scaling_governor
echo "performance" > /sys/devices/system/cpu/cpu2/cpufreq/scaling_governor
echo "performance" > /sys/devices/system/cpu/cpu3/cpufreq/scaling_governor
echo "✓ CPU governor set to performance"

# 4. Network and USB Optimizations
echo "Optimizing USB and network for CAN performance..."

# Increase USB polling rate for CAN adapters
echo 1 > /sys/module/usbcore/parameters/autosuspend_delay_ms
echo "✓ USB autosuspend optimized"

# 5. Real-time Kernel Parameters
echo "Setting real-time kernel parameters..."
sysctl -w kernel.sched_rt_period_us=1000000
sysctl -w kernel.sched_rt_runtime_us=950000  # 95% for RT tasks
sysctl -w kernel.hung_task_timeout_secs=0    # Disable hung task detection
echo "✓ Real-time scheduler parameters set"

# 6. Thermal Management
echo "Configuring thermal management..."

# Set thermal governor to powersave to prevent throttling
if [ -f /sys/class/thermal/thermal_zone0/policy ]; then
  echo "step_wise" > /sys/class/thermal/thermal_zone0/policy
fi

# Increase thermal limits (if cooling is adequate)
# echo 85000 > /sys/class/thermal/thermal_zone0/trip_point_0_temp 2>/dev/null
echo "✓ Thermal management configured"

# 7. CAN Interface Optimization
echo "Optimizing CAN interface parameters..."

# Set CAN interface buffer sizes
if [ -d /sys/class/net/can0 ]; then
  ip link set can0 txqueuelen 1000
  echo "✓ CAN buffer sizes optimized"
fi

# 8. Create persistent configuration
echo "Creating persistent configuration..."

cat > /etc/sysctl.d/99-rpi4-realtime.conf << 'EOF'
# Raspberry Pi 4 Real-time Optimizations
vm.swappiness=1
vm.dirty_ratio=5
vm.dirty_background_ratio=2
vm.dirty_expire_centisecs=500
vm.dirty_writeback_centisecs=100
kernel.sched_rt_period_us=1000000
kernel.sched_rt_runtime_us=950000
kernel.hung_task_timeout_secs=0
EOF

cat > /etc/systemd/system/rpi4-realtime.service << 'EOF'
[Unit]
Description=RPi4 Real-time optimizations
After=multi-user.target

[Service]
Type=oneshot
ExecStart=/bin/bash -c '
  # CPU governor
  echo performance > /sys/devices/system/cpu/cpu0/cpufreq/scaling_governor;
  echo performance > /sys/devices/system/cpu/cpu1/cpufreq/scaling_governor;
  echo performance > /sys/devices/system/cpu/cpu2/cpufreq/scaling_governor;
  echo performance > /sys/devices/system/cpu/cpu3/cpufreq/scaling_governor;
  
  # IRQ affinity
  for irq in /proc/irq/*/smp_affinity; do
    if [ -w "$irq" ]; then
      echo "3" > "$irq" 2>/dev/null;
    fi
  done;
  
  # USB optimization
  echo 1 > /sys/module/usbcore/parameters/autosuspend_delay_ms;
'
RemainAfterExit=yes

[Install]
WantedBy=multi-user.target
EOF

systemctl daemon-reload
systemctl enable rpi4-realtime.service
echo "✓ Persistent configuration created"

# 9. Display optimization results
echo ""
echo "=== Optimization Summary ==="
echo "CPU Governor: $(cat /sys/devices/system/cpu/cpu0/cpufreq/scaling_governor)"
echo "CPU Frequency: $(cat /sys/devices/system/cpu/cpu0/cpufreq/scaling_cur_freq) kHz"
echo "Memory locked: $(grep VmLck /proc/self/status)"
echo "RT Runtime: $(sysctl -n kernel.sched_rt_runtime_us) µs"
echo "Swap status: $(swapon --show | wc -l) active swap files"

echo ""
echo "=== Performance Monitoring Commands ==="
echo "Monitor CPU usage:     htop"
echo "Monitor RT latency:    cyclictest -p 80 -t1 -n"
echo "Monitor CAN latency:   candump -t d can0"
echo "Check IRQ affinity:    cat /proc/interrupts"

echo ""
echo "=== Next Steps ==="
echo "1. Ensure adequate cooling (heatsink + fan recommended)"
echo "2. Reboot to apply CPU isolation (if not already set)"
echo "3. Test with: ros2 launch robot_control_system robot_control.launch.py"
echo "4. Monitor performance with the commands above"

echo ""
echo "=== RPi4 Optimization Complete ==="