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
ADAM_TYPE="${1:-adam_sp}"
MOCAP_DRIVER="${2:-noitom}"

# Valid options
VALID_ADAM_TYPES=("adam_sp" "adam_u" "adam_pro")
VALID_MOCAP_DRIVERS=("noitom" "zerolab")

# Function to print colored output
print_info() {
    echo -e "${BLUE}[INFO]${NC} $1"
}

print_success() {
    echo -e "${GREEN}[SUCCESS]${NC} $1"
}

print_warning() {
    echo -e "${YELLOW}[WARNING]${NC} $1"
}

print_error() {
    echo -e "${RED}[ERROR]${NC} $1"
}

# Function to show usage
show_usage() {
    echo -e "${BLUE}PND Retarget Launch Script${NC}"
    echo ""
    echo "Usage: $0 [adam_type] [mocap_driver]"
    echo ""
    echo "Parameters:"
    echo "  adam_type     Robot type (default: adam_sp)"
    echo "                Valid options: ${VALID_ADAM_TYPES[*]}"
    echo ""
    echo "  mocap_driver  Motion capture driver (default: noitom)"
    echo "                Valid options: ${VALID_MOCAP_DRIVERS[*]}"
    echo ""
    echo "Examples:"
    echo "  $0                          # Use defaults (adam_sp + noitom)"
    echo "  $0 adam_u                   # Use adam_u with default mocap (noitom)"
    echo "  $0 adam_pro zerolab      # Use adam_pro with zerolab mocap"
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

# Function to get launch command
get_launch_command() {
    case "${ADAM_TYPE}_${MOCAP_DRIVER}" in
        "adam_sp_noitom")
            echo "ros2 launch bringup retarget_adam_sp.launch.py"
            ;;
        "adam_u_noitom")
            echo "ros2 launch bringup retarget_adam_u.launch.py"
            ;;
        "adam_pro_noitom")
            echo "ros2 launch bringup retarget_adam_pro.launch.py"
            ;;
        "adam_pro_zerolab")
            echo "ros2 launch bringup retarget_adam_pro_zerolab.launch.py"
            ;;
        *)
            print_error "Unsupported combination: ${ADAM_TYPE} + ${MOCAP_DRIVER}"
            print_warning "Currently supported combinations:"
            print_warning "  adam_sp + noitom"
            print_warning "  adam_u + noitom"
            print_warning "  adam_pro + noitom"
            print_warning "  adam_pro + zerolab"
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
    
    # Check ROS environment
    print_info "Checking ROS environment..."
    check_ros_environment
    
    # Source ROS environment
    print_info "Sourcing ROS environment..."
    source /opt/ros/humble/setup.bash
    source install/setup.bash
    export ROS_LOCALHOST_ONLY=1
    
    # Get launch command
    LAUNCH_CMD=$(get_launch_command)
    
    print_success "Environment configured successfully"
    print_info "Launching: ${LAUNCH_CMD}"
    print_info "Press Ctrl+C to stop..."
    echo ""
    
    # Execute launch command
    exec $LAUNCH_CMD
}

# Run main function
main "$@"