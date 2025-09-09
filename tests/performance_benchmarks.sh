#!/bin/bash
# ADIN2111-MK Driver Performance Benchmarking Suite
# Comprehensive performance testing and comparison framework

set -e

# Configuration
TEST_DIR="/home/murr2k/projects/delta/adin2111-mk"
DRIVER_NAME="adin2111-mk"
PRISTINE_DRIVER="adin2111-pristine"
INTERFACE="eth0"
TEST_IP="172.16.1.100"
PEER_IP="172.16.1.101"  # Configure peer for testing
RESULTS_DIR="./tests/results"
TIMESTAMP=$(date +%Y%m%d_%H%M%S)

# Performance test parameters
PACKET_SIZES=(64 128 256 512 1024 1500)
TEST_DURATION=30  # seconds
WARMUP_TIME=5     # seconds

# Color codes
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
CYAN='\033[0;36m'
NC='\033[0m'

# Logging functions
log_info() { echo -e "${BLUE}[INFO]${NC} $1"; }
log_success() { echo -e "${GREEN}[PASS]${NC} $1"; }
log_failure() { echo -e "${RED}[FAIL]${NC} $1"; }
log_warning() { echo -e "${YELLOW}[WARN]${NC} $1"; }
log_perf() { echo -e "${CYAN}[PERF]${NC} $1"; }

# Create results directory
mkdir -p "$RESULTS_DIR"

# Performance measurement functions
measure_latency() {
    local driver=$1
    local packet_size=$2
    local output_file="$RESULTS_DIR/latency_${driver}_${packet_size}_${TIMESTAMP}.txt"
    
    log_perf "Measuring latency for $driver (packet size: $packet_size bytes)"
    
    # Use ping with specific packet size
    local ping_size=$((packet_size - 28))  # Subtract IP+ICMP headers
    if [ $ping_size -lt 0 ]; then ping_size=1; fi
    
    ping -c 100 -s $ping_size -i 0.01 $TEST_IP 2>/dev/null | \
        grep "time=" | \
        sed 's/.*time=\([0-9.]*\).*/\1/' > "$output_file"
    
    if [ -s "$output_file" ]; then
        local avg_latency=$(awk '{sum+=$1; count++} END {print sum/count}' "$output_file")
        local min_latency=$(sort -n "$output_file" | head -1)
        local max_latency=$(sort -n "$output_file" | tail -1)
        
        echo "Driver: $driver, Size: $packet_size, Avg: ${avg_latency}ms, Min: ${min_latency}ms, Max: ${max_latency}ms"
        return 0
    else
        log_warning "Latency measurement failed for $driver"
        return 1
    fi
}

measure_throughput() {
    local driver=$1
    local packet_size=$2
    local direction=$3  # "tx" or "rx"
    local output_file="$RESULTS_DIR/throughput_${driver}_${direction}_${packet_size}_${TIMESTAMP}.txt"
    
    log_perf "Measuring $direction throughput for $driver (packet size: $packet_size bytes)"
    
    # Generate traffic using netperf if available, otherwise use custom method
    if command -v netperf >/dev/null 2>&1; then
        # Using netperf for precise throughput measurement
        case $direction in
            "tx")
                netperf -H $PEER_IP -l $TEST_DURATION -t TCP_STREAM -- -m $packet_size > "$output_file" 2>&1 || true
                ;;
            "rx")
                netperf -H $PEER_IP -l $TEST_DURATION -t TCP_MAERTS -- -m $packet_size > "$output_file" 2>&1 || true
                ;;
        esac
    else
        # Fallback to iperf3 or custom measurement
        if command -v iperf3 >/dev/null 2>&1; then
            case $direction in
                "tx")
                    iperf3 -c $PEER_IP -t $TEST_DURATION -l $packet_size --json > "$output_file" 2>&1 || true
                    ;;
                "rx")
                    iperf3 -c $PEER_IP -t $TEST_DURATION -l $packet_size -R --json > "$output_file" 2>&1 || true
                    ;;
            esac
        else
            # Basic measurement using dd and nc
            log_warning "Advanced throughput tools not available, using basic measurement"
            measure_basic_throughput "$packet_size" "$direction" "$output_file"
        fi
    fi
    
    # Parse results
    parse_throughput_results "$output_file" "$driver" "$packet_size" "$direction"
}

measure_basic_throughput() {
    local packet_size=$1
    local direction=$2
    local output_file=$3
    
    # Basic throughput measurement using interface statistics
    local start_rx=$(cat /sys/class/net/$INTERFACE/statistics/rx_bytes)
    local start_tx=$(cat /sys/class/net/$INTERFACE/statistics/tx_bytes)
    local start_time=$(date +%s)
    
    # Generate some traffic (ping flood for basic testing)
    ping -f -c 1000 -s $packet_size $TEST_IP >/dev/null 2>&1 &
    local ping_pid=$!
    
    sleep $TEST_DURATION
    kill $ping_pid 2>/dev/null || true
    
    local end_rx=$(cat /sys/class/net/$INTERFACE/statistics/rx_bytes)
    local end_tx=$(cat /sys/class/net/$INTERFACE/statistics/tx_bytes)
    local end_time=$(date +%s)
    
    local duration=$((end_time - start_time))
    local rx_bytes=$((end_rx - start_rx))
    local tx_bytes=$((end_tx - start_tx))
    
    echo "Duration: ${duration}s" > "$output_file"
    echo "RX Bytes: $rx_bytes" >> "$output_file"
    echo "TX Bytes: $tx_bytes" >> "$output_file"
    echo "RX Throughput: $((rx_bytes * 8 / duration / 1000000)) Mbps" >> "$output_file"
    echo "TX Throughput: $((tx_bytes * 8 / duration / 1000000)) Mbps" >> "$output_file"
}

parse_throughput_results() {
    local output_file=$1
    local driver=$2
    local packet_size=$3
    local direction=$4
    
    if [ -s "$output_file" ]; then
        log_success "Throughput measurement completed for $driver ($direction, ${packet_size}B)"
        # Add parsing logic based on the tool used
        grep -E "(Mbps|throughput)" "$output_file" | head -3
    else
        log_warning "Throughput measurement failed for $driver"
    fi
}

measure_cpu_usage() {
    local driver=$1
    local test_type=$2
    local output_file="$RESULTS_DIR/cpu_usage_${driver}_${test_type}_${TIMESTAMP}.txt"
    
    log_perf "Measuring CPU usage for $driver during $test_type"
    
    # Start CPU monitoring
    sar -u 1 $TEST_DURATION > "$output_file" &
    local sar_pid=$!
    
    # Run the test workload
    case $test_type in
        "idle")
            sleep $TEST_DURATION
            ;;
        "traffic")
            ping -f -c 10000 $TEST_IP >/dev/null 2>&1 || true
            ;;
        "stress")
            # Generate high traffic load
            for i in {1..5}; do
                ping -f -c 2000 $TEST_IP >/dev/null 2>&1 &
            done
            wait
            ;;
    esac
    
    wait $sar_pid
    
    # Parse CPU usage
    local avg_cpu=$(tail -n +4 "$output_file" | head -n -2 | awk '{sum+=(100-$8)} END {print sum/NR}')
    log_perf "Average CPU usage: ${avg_cpu}%"
    
    echo "$avg_cpu"
}

measure_memory_usage() {
    local driver=$1
    local output_file="$RESULTS_DIR/memory_usage_${driver}_${TIMESTAMP}.txt"
    
    log_perf "Measuring memory usage for $driver"
    
    # Baseline memory
    local mem_before=$(cat /proc/meminfo | grep MemAvailable | awk '{print $2}')
    
    # Load driver and run traffic
    ifconfig $INTERFACE $TEST_IP up
    sleep 5
    
    # Generate traffic for memory stress
    ping -f -c 5000 $TEST_IP >/dev/null 2>&1 || true
    
    # Measure memory after
    local mem_after=$(cat /proc/meminfo | grep MemAvailable | awk '{print $2}')
    local mem_used=$((mem_before - mem_after))
    
    echo "Memory before: ${mem_before} KB" > "$output_file"
    echo "Memory after: ${mem_after} KB" >> "$output_file"
    echo "Memory used: ${mem_used} KB" >> "$output_file"
    
    log_perf "Memory usage: ${mem_used} KB"
    echo "$mem_used"
}

measure_interrupt_performance() {
    local driver=$1
    local output_file="$RESULTS_DIR/interrupts_${driver}_${TIMESTAMP}.txt"
    
    log_perf "Measuring interrupt performance for $driver"
    
    # Capture interrupt stats before
    cat /proc/interrupts > "${output_file}.before"
    
    # Generate traffic
    ping -f -c 1000 $TEST_IP >/dev/null 2>&1 || true
    
    # Capture interrupt stats after
    cat /proc/interrupts > "${output_file}.after"
    
    # Calculate interrupt rate
    local irq_count_before=$(grep -E "(adin|eth)" "${output_file}.before" | awk '{print $2}' | head -1)
    local irq_count_after=$(grep -E "(adin|eth)" "${output_file}.after" | awk '{print $2}' | head -1)
    
    if [ -n "$irq_count_before" ] && [ -n "$irq_count_after" ]; then
        local irq_diff=$((irq_count_after - irq_count_before))
        echo "Interrupt count: $irq_diff" > "$output_file"
        log_perf "Interrupt count during test: $irq_diff"
    else
        log_warning "Could not measure interrupt performance"
    fi
}

run_driver_benchmarks() {
    local driver=$1
    local driver_path=$2
    
    log_info "Running performance benchmarks for $driver"
    
    # Load driver
    if [ -f "$driver_path" ]; then
        insmod "$driver_path"
        sleep 5
    else
        log_failure "Driver $driver_path not found"
        return 1
    fi
    
    # Configure interface
    ifconfig $INTERFACE $TEST_IP up
    sleep 3
    
    # Warmup
    log_info "Warming up for ${WARMUP_TIME} seconds..."
    ping -c $WARMUP_TIME $TEST_IP >/dev/null 2>&1 || true
    
    # Memory usage baseline
    local memory_usage=$(measure_memory_usage "$driver")
    
    # CPU usage tests
    local cpu_idle=$(measure_cpu_usage "$driver" "idle")
    local cpu_traffic=$(measure_cpu_usage "$driver" "traffic")
    
    # Interrupt performance
    measure_interrupt_performance "$driver"
    
    # Latency tests for different packet sizes
    for size in "${PACKET_SIZES[@]}"; do
        measure_latency "$driver" "$size" || true
    done
    
    # Throughput tests (if peer available)
    if ping -c 1 -W 2 $PEER_IP >/dev/null 2>&1; then
        for size in "${PACKET_SIZES[@]}"; do
            measure_throughput "$driver" "$size" "tx" || true
            measure_throughput "$driver" "$size" "rx" || true
        done
    else
        log_warning "Peer $PEER_IP not available for throughput testing"
    fi
    
    # Store summary results
    {
        echo "=== Performance Summary for $driver ==="
        echo "Timestamp: $(date)"
        echo "Memory Usage: ${memory_usage} KB"
        echo "CPU Usage (idle): ${cpu_idle}%"
        echo "CPU Usage (traffic): ${cpu_traffic}%"
        echo "=================================="
    } > "$RESULTS_DIR/summary_${driver}_${TIMESTAMP}.txt"
    
    # Clean up
    ifconfig $INTERFACE down
    rmmod "$driver" 2>/dev/null || true
    sleep 2
}

compare_results() {
    log_info "Comparing performance results..."
    
    local comparison_file="$RESULTS_DIR/comparison_${TIMESTAMP}.txt"
    
    {
        echo "=== ADIN2111 Driver Performance Comparison ==="
        echo "Generated: $(date)"
        echo ""
        
        echo "=== Summary Comparison ==="
        if [ -f "$RESULTS_DIR/summary_${DRIVER_NAME}_${TIMESTAMP}.txt" ]; then
            echo "--- MK Driver Results ---"
            cat "$RESULTS_DIR/summary_${DRIVER_NAME}_${TIMESTAMP}.txt"
            echo ""
        fi
        
        if [ -f "$RESULTS_DIR/summary_${PRISTINE_DRIVER}_${TIMESTAMP}.txt" ]; then
            echo "--- Pristine Driver Results ---"
            cat "$RESULTS_DIR/summary_${PRISTINE_DRIVER}_${TIMESTAMP}.txt"
            echo ""
        fi
        
        echo "=== Detailed Analysis ==="
        
        # Latency comparison
        echo "--- Latency Comparison ---"
        for size in "${PACKET_SIZES[@]}"; do
            local mk_latency_file="$RESULTS_DIR/latency_${DRIVER_NAME}_${size}_${TIMESTAMP}.txt"
            local pristine_latency_file="$RESULTS_DIR/latency_${PRISTINE_DRIVER}_${size}_${TIMESTAMP}.txt"
            
            if [ -f "$mk_latency_file" ] && [ -f "$pristine_latency_file" ]; then
                local mk_avg=$(awk '{sum+=$1} END {print sum/NR}' "$mk_latency_file")
                local pristine_avg=$(awk '{sum+=$1} END {print sum/NR}' "$pristine_latency_file")
                
                echo "Packet Size ${size}B: MK=${mk_avg}ms, Pristine=${pristine_avg}ms"
            fi
        done
        
        echo ""
        echo "=== Performance Verdict ==="
        
        # Add automated comparison logic here
        echo "Detailed analysis requires manual review of individual test files."
        echo "All test results stored in: $RESULTS_DIR"
        
    } > "$comparison_file"
    
    log_success "Performance comparison saved to: $comparison_file"
    cat "$comparison_file"
}

generate_performance_report() {
    local report_file="$RESULTS_DIR/performance_report_${TIMESTAMP}.html"
    
    log_info "Generating HTML performance report..."
    
    cat > "$report_file" << 'EOF'
<!DOCTYPE html>
<html>
<head>
    <title>ADIN2111-MK Driver Performance Report</title>
    <style>
        body { font-family: Arial, sans-serif; margin: 20px; }
        .header { background-color: #f0f0f0; padding: 20px; border-radius: 5px; }
        .section { margin: 20px 0; padding: 15px; border: 1px solid #ddd; }
        .pass { color: green; font-weight: bold; }
        .fail { color: red; font-weight: bold; }
        .warn { color: orange; font-weight: bold; }
        table { border-collapse: collapse; width: 100%; }
        th, td { border: 1px solid #ddd; padding: 8px; text-align: left; }
        th { background-color: #f2f2f2; }
        pre { background-color: #f8f8f8; padding: 10px; overflow-x: auto; }
    </style>
</head>
<body>
    <div class="header">
        <h1>🧪 ADIN2111-MK Driver Performance Report</h1>
        <p><strong>Generated:</strong> $(date)</p>
        <p><strong>Test Environment:</strong> Ubuntu 6.6.48-stm32mp</p>
        <p><strong>Driver:</strong> adin2111-mk.c vs adin2111-pristine.c</p>
    </div>

    <div class="section">
        <h2>Executive Summary</h2>
        <p>This report contains comprehensive performance analysis of the ADIN2111-MK driver
        modifications for single interface eth0 operation compared to the original pristine driver.</p>
    </div>

    <div class="section">
        <h2>Test Results Overview</h2>
        <!-- Results will be populated by script -->
        <div id="results-placeholder">
            <p>Performance test results are stored in individual files in the results directory.</p>
            <p>For detailed analysis, please review:</p>
            <ul>
EOF

    # Add file listings
    for file in "$RESULTS_DIR"/*_${TIMESTAMP}.txt; do
        if [ -f "$file" ]; then
            echo "                <li>$(basename "$file")</li>" >> "$report_file"
        fi
    done

    cat >> "$report_file" << 'EOF'
            </ul>
        </div>
    </div>

    <div class="section">
        <h2>Recommendations</h2>
        <ul>
            <li>Review latency measurements for any significant increases</li>
            <li>Verify throughput performance meets requirements</li>
            <li>Monitor CPU and memory usage under various loads</li>
            <li>Validate that single interface operation doesn't impact performance</li>
        </ul>
    </div>

    <div class="section">
        <h2>Next Steps</h2>
        <ol>
            <li>Analyze detailed performance metrics in individual result files</li>
            <li>Perform extended testing under production workloads</li>
            <li>Validate performance under stress conditions</li>
            <li>Compare results with performance requirements</li>
        </ol>
    </div>
</body>
</html>
EOF

    log_success "HTML report generated: $report_file"
}

# Main execution
main() {
    echo "=================================================================="
    echo "🧪 ADIN2111-MK Driver Performance Benchmarking Suite"
    echo "=================================================================="
    echo ""
    
    # Setup
    cd "$TEST_DIR"
    
    # Check for required tools
    log_info "Checking for performance testing tools..."
    for tool in sar ping tcpdump; do
        if command -v "$tool" >/dev/null 2>&1; then
            log_success "$tool available"
        else
            log_warning "$tool not available (some tests may be limited)"
        fi
    done
    
    # Check if drivers exist
    if [ ! -f "src/${DRIVER_NAME}.ko" ]; then
        log_info "Building MK driver..."
        make -C src/ clean && make -C src/
    fi
    
    # Run benchmarks for MK driver
    if [ -f "src/${DRIVER_NAME}.ko" ]; then
        run_driver_benchmarks "$DRIVER_NAME" "src/${DRIVER_NAME}.ko"
    else
        log_failure "MK driver not found"
        exit 1
    fi
    
    # Run benchmarks for pristine driver (if available)
    if [ -f "src/${PRISTINE_DRIVER}.ko" ]; then
        run_driver_benchmarks "$PRISTINE_DRIVER" "src/${PRISTINE_DRIVER}.ko"
        compare_results
    else
        log_warning "Pristine driver not available for comparison"
    fi
    
    # Generate reports
    generate_performance_report
    
    echo ""
    echo "=================================================================="
    echo "🧪 Performance Benchmarking Complete"
    echo "=================================================================="
    echo -e "${GREEN}Results saved to: $RESULTS_DIR${NC}"
    echo -e "${CYAN}Review the following files for detailed analysis:${NC}"
    ls -la "$RESULTS_DIR"/*_${TIMESTAMP}.*
    echo ""
}

# Execute main function
main "$@"