#!/bin/bash
set -e
echo "Installing spdlog"

# Get install directory as script argument
INSTALL_DIR=$1


# Clone source code
if [ ! -d spdlog ] ; then
  git clone https://github.com/gabime/spdlog.git
  cd spdlog
else
  cd spdlog
fi

# Use specific version
git fetch -p
git checkout v1.14.1
git submodule update --init --recursive

# Configure CMake
mkdir -p build && cd build
cmake .. -DCMAKE_BUILD_TYPE=Release \
        -DBUILD_SHARED_LIBS=OFF \
        -DCMAKE_INSTALL_PREFIX=$INSTALL_DIR \
        -DCMAKE_PREFIX_PATH=$INSTALL_DIR \
        -DSPDLOG_BUILD_TESTS=OFF

# Build and install
cmake --build . --target install --config Release -j $NUM_JOBS

echo "Installed spdlog"
