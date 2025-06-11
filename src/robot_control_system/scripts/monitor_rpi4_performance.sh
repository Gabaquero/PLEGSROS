#!/bin/bash
# Raspberry Pi 4 Real-Time Performance Monitor
# Monitors system performance during robot control operation

echo "=== RPi4 Real-Time Performance Monitor ==="
echo "Press Ctrl+C to stop monitoring"
echo ""

# Function to get CPU temperature
get_cpu_temp() {
    if [ -f /sys/class/thermal/thermal_zone0/temp ]; then
        temp=$(cat /sys/class/thermal/thermal_zone0/temp)
        echo "scale=1; $temp/1000" | bc
    else
        echo "N/A"
    fi
}

# Function to get CPU frequency
get_cpu_freq() {
    if [ -f /sys/devices/system/cpu/cpu0/cpufreq/scaling_cur_freq ]; then
        freq=$(cat /sys/devices/system/cpu/cpu0/cpufreq/scaling_cur_freq)
        echo "scale=0; $freq/1000" | bc
    else
        echo "N/A"
    fi
}

# Function to check if ROS nodes are running
check_ros_nodes() {
    if pgrep -f "reception_node" > /dev/null; then
        echo "✓"
    else
        echo "✗"
    fi
}

# Function to get memory usage
get_memory_usage() {
    free | awk 'NR==2{printf "%.1f%%", $3*100/$2}'
}

# Function to get CAN stats
get_can_stats() {
    if [ -f /sys/class/net/can0/statistics/rx_packets ]; then
        rx=$(cat /sys/class/net/can0/statistics/rx_packets)
        tx=$(cat /sys/class/net/can0/statistics/tx_packets)
        echo "RX:$rx TX:$tx"
    else
        echo "CAN0:N/A"
    fi
}

# Function to check RT latency with cyclictest (if available)
check_rt_latency() {
    if command -v cyclictest >/dev/null 2>&1; then
        # Run cyclictest for 1 second and get max latency
        timeout 1s cyclictest -p 80 -t1 -n -q 2>/dev/null | grep -o "Max:[[:space:]]*[0-9]*" | awk '{print $2}'
    else
        echo "N/A"
    fi
}

# Main monitoring loop
while true; do
    clear
    echo "=== RPi4 Real-Time Performance Monitor ==="
    echo "$(date)"
    echo ""
    
    # System Overview
    echo "┌─ System Overview ─────────────────────────────────────┐"
    printf "│ CPU Temp: %6s°C │ CPU Freq: %8s MHz │ Gov: %11s │\n" \
        "$(get_cpu_temp)" "$(get_cpu_freq)" "$(cat /sys/devices/system/cpu/cpu0/cpufreq/scaling_governor 2>/dev/null || echo 'N/A')"
    printf "│ Memory:   %6s   │ Load Avg: %8s     │ Uptime: %8s │\n" \
        "$(get_memory_usage)" "$(uptime | awk -F'load average:' '{print $2}' | awk '{print $1}' | tr -d ',')" \
        "$(uptime | awk '{print $3}' | tr -d ',')"
    echo "└───────────────────────────────────────────────────────┘"
    echo ""
    
    # CPU Core Usage
    echo "┌─ CPU Core Usage ──────────────────────────────────────┐"
    top -bn1 | grep "Cpu" | head -4 | while IFS= read -r line; do
        core=$(echo "$line" | awk '{print $1}')
        usage=$(echo "$line" | awk '{print $2}' | tr -d '%us,')
        printf "│ %-8s: %5.1f%% │" "$core" "$usage"
    done
    echo ""
    echo "└───────────────────────────────────────────────────────┘"
    echo ""
    
    # Real-Time Performance
    echo "┌─ Real-Time Performance ───────────────────────────────┐"
    rt_latency=$(check_rt_latency)
    printf "│ RT Max Latency: %8s μs │ ROS Nodes: %12s │\n" \
        "${rt_latency:-"N/A"}" "$(check_ros_nodes)"
    
    # Check for process priority
    pd_prio=$(ps -eo pid,comm,rtprio | grep "pd_controller" | head -1 | awk '{print $3}')
    reception_prio=$(ps -eo pid,comm,rtprio | grep "reception" | head -1 | awk '{print $3}')
    printf "│ PD Priority:    %8s    │ RX Priority: %8s    │\n" \
        "${pd_prio:-"N/A"}" "${reception_prio:-"N/A"}"
    echo "└───────────────────────────────────────────────────────┘"
    echo ""
    
    # CAN Interface Status
    echo "┌─ CAN Interface Status ────────────────────────────────┐"
    if [ -d /sys/class/net/can0 ]; then
        can_state=$(cat /sys/class/net/can0/operstate 2>/dev/null || echo "down")
        can_stats=$(get_can_stats)
        printf "│ CAN0 State: %10s │ Packets: %15s │\n" "$can_state" "$can_stats"
        
        # CAN error counters
        if [ -f /sys/class/net/can0/statistics/rx_errors ]; then
            rx_err=$(cat /sys/class/net/can0/statistics/rx_errors)
            tx_err=$(cat /sys/class/net/can0/statistics/tx_errors)
            printf "│ RX Errors:  %10s │ TX Errors: %12s │\n" "$rx_err" "$tx_err"
        fi
    else
        printf "│ CAN0: Not available                                  │\n"
    fi
    echo "└───────────────────────────────────────────────────────┘"
    echo ""
    
    # Process Information
    echo "┌─ Robot Control Processes ─────────────────────────────┐"
    ps -eo pid,comm,pcpu,pmem,rtprio --sort=-pcpu | grep -E "(reception|pd_controller|command_sender|trajectory)" | head -6 | \
    while IFS= read -r line; do
        printf "│ %-50s │\n" "$line"
    done
    echo "└───────────────────────────────────────────────────────┘"
    echo ""
    
    # Performance Warnings
    echo "┌─ Performance Warnings ────────────────────────────────┐"
    temp=$(get_cpu_temp)
    if [ "$temp" != "N/A" ] && [ "$(echo "$temp > 70" | bc)" -eq 1 ]; then
        echo "│ ⚠️  HIGH TEMPERATURE: ${temp}°C - Check cooling!        │"
    fi
    
    load=$(uptime | awk -F'load average:' '{print $2}' | awk '{print $1}' | tr -d ',')
    if [ "$(echo "$load > 3.0" | bc 2>/dev/null)" -eq 1 ] 2>/dev/null; then
        echo "│ ⚠️  HIGH LOAD: $load - System may be overloaded!       │"
    fi
    
    if [ "$pd_prio" = "-" ] || [ -z "$pd_prio" ]; then
        echo "│ ⚠️  NO RT PRIORITY: PD controller not real-time!       │"
    fi
    
    if [ "$(check_ros_nodes)" = "✗" ]; then
        echo "│ ⚠️  ROS NODES DOWN: Robot control not running!         │"
    fi
    
    # Check if no warnings
    if [ "$temp" != "N/A" ] && [ "$(echo "$temp <= 70" | bc)" -eq 1 ] && \
       [ "$(echo "$load <= 3.0" | bc 2>/dev/null)" -eq 1 ] 2>/dev/null && \
       [ "$pd_prio" != "-" ] && [ -n "$pd_prio" ] && \
       [ "$(check_ros_nodes)" = "✓" ]; then
        echo "│ ✅ All systems nominal - Performance optimal!          │"
    fi
    echo "└───────────────────────────────────────────────────────┘"
    echo ""
    
    echo "Monitoring... (Ctrl+C to stop)"
    sleep 2
done