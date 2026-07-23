#!/bin/sh
# This script builds from source and installs all dependencies of flexiv_tdk.

# Absolute path of this script
SCRIPTPATH="$(dirname $(readlink -f $0))"
set -e

# Initialize variables
INSTALL_DIR=""
NUM_JOBS=4

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
    echo "  -h, --help                Show this help message."
    echo ""
    echo "Example:"
    echo "  $0 /opt/flexiv -j 8"
}

# Parse arguments
while [ "$#" -gt 0 ]; do
    case "$1" in
        -h|--help)
            print_usage
            exit 0
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

# Clone all dependencies in a subfolder
mkdir -p cloned && cd cloned

# Build and install all dependencies to INSTALL_DIR
bash $SCRIPTPATH/scripts/install_flexiv_rdk.sh $INSTALL_DIR $NUM_JOBS

echo ">>>>>>>>>> Finished <<<<<<<<<<"