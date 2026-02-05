#!/bin/bash

# PND Retarget Launch Script
# Description: Launch retargeting nodes for different ADAM robot types with various mocap drivers
# Usage: ./run.sh [adam_type] [mocap_driver]

set -e  # Exit on any error

# Color codes for output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m' # No Color

# Default values
ADAM_TYPE="${1:-adam_pro}"
MOCAP_DRIVER="${2:-noitom}"
ALGORITHM="${3:-mink}"

# Valid options
VALID_ADAM_TYPES=("adam_sp" "adam_u" "adam_pro")
VALID_MOCAP_DRIVERS=("noitom" "zerolab" "vr")
VALID_ALGORITHMS=("pinocchio" "mink")

# Function to print colored output
print_info() {
    echo -e "${BLUE}[INFO]${NC} $1"
}

print_success() {
    echo -e "${GREEN}[SUCCESS]${NC} $1"
}

print_warning() {
    echo -e "${YELLOW}[WARNING]${NC} $1" >&2
}

print_error() {
    echo -e "${RED}[ERROR]${NC} $1" >&2
}

# Function to show usage
show_usage() {
    echo -e "${BLUE}PND Retarget Launch Script${NC}"
    echo ""
    echo "Usage: $0 [adam_type] [mocap_driver] [algorithm]"
    echo ""
    echo "Parameters:"
    echo "  adam_type     Robot type (default: adam_sp)"
    echo "                Valid options: ${VALID_ADAM_TYPES[*]}"
    echo ""
    echo "  mocap_driver  Motion capture driver (default: noitom)"
    echo "                Valid options: ${VALID_MOCAP_DRIVERS[*]}"
    echo ""
    echo "  algorithm     Algorithm (default: pinocchio)"
    echo "                Valid options: ${VALID_ALGORITHMS[*]}"
    echo ""
    echo "Examples:"
    echo "  $0                          # Use defaults (adam_sp + noitom)"
    echo "  $0 adam_u                   # Use adam_u with default mocap (noitom)"
    echo "  $0 adam_pro zerolab      # Use adam_pro with zerolab mocap"
    echo "  $0 adam_u vr mink      # Use adam_u with vr mocap and mink algorithm"
    echo ""
}

# Function to validate parameters
validate_adam_type() {
    for valid_type in "${VALID_ADAM_TYPES[@]}"; do
        if [[ "$ADAM_TYPE" == "$valid_type" ]]; then
            return 0
        fi
    done
    return 1
}

validate_mocap_driver() {
    for valid_driver in "${VALID_MOCAP_DRIVERS[@]}"; do
        if [[ "$MOCAP_DRIVER" == "$valid_driver" ]]; then
            return 0
        fi
    done
    return 1
}

validate_algorithm() {
    for valid_algorithm in "${VALID_ALGORITHMS[@]}"; do
        if [[ "$ALGORITHM" == "$valid_algorithm" ]]; then
            return 0
        fi
    done
    return 1
}
# Function to check if ROS environment is properly sourced
check_ros_environment() {
    if [ ! -f "/opt/ros/humble/setup.bash" ]; then
        print_error "ROS Humble not found at /opt/ros/humble/setup.bash"
        exit 1
    fi
    
    if [ ! -f "install/setup.bash" ]; then
        print_error "Local install/setup.bash not found. Please build the workspace first."
        exit 1
    fi
}

# Function to check if Caddy is installed
check_caddy_installed() {
    if ! command -v caddy &> /dev/null; then
        return 1
    fi
    return 0
}

# Function to get launch command
get_launch_command() {
    case "${ADAM_TYPE}_${MOCAP_DRIVER}_${ALGORITHM}" in
        "adam_sp_noitom_pinocchio")
            echo "ros2 launch bringup pinocchio-adam_sp-noitom.launch.py"
            ;;
        "adam_u_noitom_pinocchio")
            echo "ros2 launch bringup pinocchio-adam_u-noitom.launch.py"
            ;;
        "adam_pro_noitom_pinocchio")
            echo "ros2 launch bringup pinocchio-adam_pro-noitom.launch.py"
            ;;
        "adam_pro_zerolab_pinocchio")
            echo "ros2 launch bringup pinocchio-adam_pro-zerolab.launch.py"
            ;;
        "adam_u_vr_mink")
            echo "ros2 launch bringup mink-adam_u-webvr.launch.py"
            ;;
        "adam_pro_noitom_mink")
            echo "ros2 launch bringup mink-adam_pro-noitom.launch.py"
            # echo "ros2 launch bringup mink_adam_pro_noitom.launch.py visual:=rviz2"
            ;;
        *)
            print_error "Unsupported combination: ${ADAM_TYPE} + ${MOCAP_DRIVER} + ${ALGORITHM}"
            print_warning "Currently supported combinations:"
            print_warning "  adam_sp + noitom + pinocchio"
            print_warning "  adam_u + noitom + pinocchio"
            print_warning "  adam_u + vr + mink"
            print_warning "  adam_pro + noitom + pinocchio"
            print_warning "  adam_pro + noitom + mink"
            print_warning "  adam_pro + zerolab + pinocchio"
            exit 1
            ;;
    esac
}

# Main execution
main() {
    # Handle help request
    if [[ "$1" == "-h" || "$1" == "--help" || "$1" == "help" ]]; then
        show_usage
        exit 0
    fi
    
    print_info "Starting PND Retarget with parameters:"
    print_info "  ADAM Type: ${ADAM_TYPE}"
    print_info "  Mocap Driver: ${MOCAP_DRIVER}"
    print_info "  Algorithm: ${ALGORITHM}"
    # Validate parameters
    if ! validate_adam_type; then
        print_error "Invalid adam_type: ${ADAM_TYPE}"
        print_error "Valid options: ${VALID_ADAM_TYPES[*]}"
        echo ""
        show_usage
        exit 1
    fi
    
    if ! validate_mocap_driver; then
        print_error "Invalid mocap_driver: ${MOCAP_DRIVER}"
        print_error "Valid options: ${VALID_MOCAP_DRIVERS[*]}"
        echo ""
        show_usage
        exit 1
    fi

    if ! validate_algorithm; then
        print_error "Invalid algorithm: ${ALGORITHM}"
        print_error "Valid options: ${VALID_ALGORITHMS[*]}"
        echo ""
        show_usage
        exit 1
    fi
    # Check ROS environment
    print_info "Checking ROS environment..."
    check_ros_environment
    
    # Source ROS environment
    print_info "Sourcing ROS environment..."
    source /opt/ros/humble/setup.bash
    source install/setup.bash
    export ROS_LOCALHOST_ONLY=1
    source .venv/bin/activate || true
    
    # Get launch command
    LAUNCH_CMD=$(get_launch_command)
    
    # Start Caddy in background for adam_u_vr_mink mode
    if [[ "${ADAM_TYPE}_${MOCAP_DRIVER}_${ALGORITHM}" == "adam_u_vr_mink" ]]; then
        SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
        CADDY_SCRIPT="${SCRIPT_DIR}/scripts/start_caddy.sh"
        
        # Check if Caddy is installed
        if ! check_caddy_installed; then
            print_error "Caddy is not installed or not in PATH"
            print_info "Please install Caddy first. You can install it from https://caddyserver.com/docs/install#debian-ubuntu-raspbian"
            exit 1
        fi
        
        if [ -f "$CADDY_SCRIPT" ]; then
            print_info "Starting Caddy in background..."
            CADDY_LOG=$(mktemp)
            nohup bash "$CADDY_SCRIPT" > "$CADDY_LOG" 2>&1 &
            CADDY_PID=$!
            print_success "Caddy started in background (PID: $CADDY_PID)"
        else
            print_error "Caddy script not found at: $CADDY_SCRIPT"
        fi
    fi
    
    print_success "Environment configured successfully"
    print_info "Launching: ${LAUNCH_CMD}"
    print_info "Press Ctrl+C to stop..."
    echo ""
    
    # Execute launch command
    exec $LAUNCH_CMD
}

# Run main function
main "$@"