#!/bin/sh
# This script builds from source and installs all dependencies of flexiv_tdk.

# Absolute path of this script
SCRIPTPATH="$(dirname $(readlink -f $0))"
set -e

# Initialize variables
INSTALL_DIR=""
NUM_JOBS=4
BUILD_FOR_JAZZY=""

# Function to print usage
print_usage() {
    echo "Usage: $0 <install_directory_path> [options]"
    echo ""
    echo "Arguments:"
    echo "  install_directory_path    Directory to install all dependencies (required)."
    echo "                            Should be the same as the install directory of flexiv_tdk."
    echo ""
    echo "Options:"
    echo "  -j, --jobs <num>          Number of parallel build jobs (default: 4)."
    echo "  --ros2-jazzy              Set BUILD_FOR_JAZZY=1 for ROS2 Jazzy environment."
    echo "  -h, --help                Show this help message."
    echo ""
    echo "Example:"
    echo "  $0 /opt/flexiv --ros2-jazzy -j 8"
}

# Parse arguments
while [ "$#" -gt 0 ]; do
    case "$1" in
        -h|--help)
            print_usage
            exit 0
            ;;
        --ros2-jazzy)
            export BUILD_FOR_JAZZY=1
            echo "Detected --ros2-jazzy flag: BUILD_FOR_JAZZY set to 1"
            shift
            ;;
        -j|--jobs)
            if [ -z "$2" ] || [ "${2#-}" != "$2" ]; then
                echo "Error: -j/--jobs requires a number argument."
                print_usage
                exit 1
            fi
            NUM_JOBS=$2
            shift 2
            ;;
        -*)
            echo "Error: Unknown option $1"
            print_usage
            exit 1
            ;;
        *)
            # Positional argument (install directory)
            if [ -z "$INSTALL_DIR" ]; then
                INSTALL_DIR=$1
            else
                echo "Error: Unexpected argument $1"
                print_usage
                exit 1
            fi
            shift
            ;;
    esac
done

# Check required arguments
if [ -z "$INSTALL_DIR" ]; then
    echo "Error: missing required argument [install_directory_path]"
    print_usage
    exit 1
fi

echo "Dependencies will be installed to: $INSTALL_DIR"
echo "Number of parallel build jobs: $NUM_JOBS"
if [ -n "$BUILD_FOR_JAZZY" ]; then
    echo "ROS2 Jazzy mode enabled (BUILD_FOR_JAZZY=1)"
fi

# Clone all dependencies in a subfolder
mkdir -p cloned && cd cloned

# Build and install all dependencies to INSTALL_DIR
bash $SCRIPTPATH/scripts/install_flexiv_rdk.sh $INSTALL_DIR $NUM_JOBS

echo ">>>>>>>>>> Finished <<<<<<<<<<"