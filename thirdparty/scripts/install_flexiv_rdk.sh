#!/bin/bash
set -e
echo "Installing flexiv_rdk"

# Use a specific version
GIT_TAG=release/v1.7

# Get install directory and number of parallel build jobs as script arguments
INSTALL_DIR=$1
NUM_JOBS=$2

# Clone source code with only 1 layer of history
if [ ! -d flexiv_rdk ] ; then
  git clone https://github.com/flexivrobotics/flexiv_rdk.git --depth 1 --branch $GIT_TAG
  cd flexiv_rdk
else
  cd flexiv_rdk
fi

# Save path to flexiv_rdk root
ROOT_DIR=$(pwd)

# Build and install nested dependencies
cd thirdparty
bash build_and_install_dependencies.sh $INSTALL_DIR $NUM_JOBS

# Configure CMake
cd $ROOT_DIR
mkdir -p build && cd build
cmake .. -DCMAKE_BUILD_TYPE=Release \
         -DCMAKE_INSTALL_PREFIX=$INSTALL_DIR

# Build and install
cmake --build . --target install --config Release -j $NUM_JOBS

echo "Installed flexiv_rdk"
