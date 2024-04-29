#!/bin/bash
set -e
echo "Installing boost"

# Get install directory as script argument
INSTALL_DIR=$1

# Download source code
if [ ! -d boost_1_74_0 ] ; then
  # download is faster than clone
  URL="https://boostorg.jfrog.io/artifactory/main/release/1.72.0/source/boost_1_74_0.tar.bz2"
  echo "-- Downloading: $URL"
  wget $URL --no-clobber --quiet --show-progress --progress=bar:force 2>&1
  # Unzip
  echo "-- Extracting: boost_1_74_0.tar.bz2"
  tar --bzip2 -xf "boost_1_74_0.tar.bz2"
  cd boost_1_74_0
else
  cd boost_1_74_0
fi

# Build and install
./bootstrap.sh --prefix=$INSTALL_DIR --with-libraries=filesystem
./b2 variant=release install

echo "Installed boost"
