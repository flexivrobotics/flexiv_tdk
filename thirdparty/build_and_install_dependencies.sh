#!/bin/sh
# This script builds from source and installs all dependencies of flexiv_tdk.

# Absolute path of this script
SCRIPTPATH="$(dirname $(readlink -f $0))"
set -e

# Check script arguments
if [ "$#" -lt 1 ]; then
    echo "Error: invalid script argument"
    echo "Required argument: [install_directory_path]"
    echo "    install_directory_path: directory to install all dependencies, should be the same as the install directory of flexiv_tdk"
    echo "Optional argument: [num_parallel_jobs]"
    echo "    num_parallel_jobs: number of parallel jobs used to build, if not specified, the number of CPU cores will be used"
    exit
fi

# Get dependencies install directory from script argument, should be the same as the install directory of flexiv_tdk
INSTALL_DIR=$1
echo "Dependencies will be installed to: $INSTALL_DIR"

# Use specified number for parallel build jobs, otherwise use number of cores 
if [ -n "$2" ] ;then
    export NUM_JOBS=$2
else
    export NUM_JOBS=$(nproc)
fi
echo "Number of parallel build jobs: $NUM_JOBS"


# Clone all dependencies in a subfolder
mkdir -p cloned && cd cloned

# Build and install all dependencies to INSTALL_DIR
bash $SCRIPTPATH/scripts/install_rdk.sh $INSTALL_DIR
echo ">>>>>>>>>> Finished <<<<<<<<<<"
