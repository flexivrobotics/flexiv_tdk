#!/bin/bash
set -e
echo "Installing flexiv_rdk"

# Get install directory as script argument
INSTALL_DIR=$1


# Clone source code
if [ ! -d flexiv_rdk ] ; then
  git clone https://github.com/flexivrobotics/flexiv_rdk.git
  cd flexiv_rdk
else
  cd flexiv_rdk
fi

# Set absolute path of flexiv_rdk
RDK_DIR="$PWD"

# Use specific version
git fetch -p
git checkout v1.4

# Build and install dependencies
cd $RDK_DIR/thirdparty
bash build_and_install_dependencies.sh $INSTALL_DIR

cd $RDK_DIR
# Configure CMake
mkdir -p build && cd build
cmake .. -DCMAKE_BUILD_TYPE=Release \
        -DBUILD_SHARED_LIBS=OFF \
        -DCMAKE_INSTALL_PREFIX=$INSTALL_DIR \
        -DCMAKE_PREFIX_PATH=$INSTALL_DIR 

# Build and install
cmake --build . --target install --config Release -j $NUM_JOBS

echo "Installed flexiv_rdk"
