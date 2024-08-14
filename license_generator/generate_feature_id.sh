#!/bin/bash
# Set absolute path of script
SCRIPT_DIR="$(dirname $(readlink -f $0))"

# Check current OS type
OS_NAME=$(grep "^NAME" /etc/os-release | awk -F '=' '{print $2}' | tr -d '"')

if [[ "$OS_NAME" != "Ubuntu" ]]; then
    echo "Unsupported OS: $OS_NAME"
    exit 1
fi

# Get version info
OS_VERSION=$(grep "VERSION_ID" /etc/os-release | awk -F '=' '{print $2}' | tr -d '"')

# Check version
if [[ "$OS_VERSION" == "20.04" ]]; then
    echo "Generating feature_id on Ubuntu 20.04 ..."
    if [[ ! -x "$SCRIPT_DIR/generator_20" ]]; then
        echo "generator_20 is not executable. Please run: chmod +x $SCRIPT_DIR/generator_20"
        exit 1
    fi
    "$SCRIPT_DIR/generator_20"
elif [[ "$OS_VERSION" == "22.04" ]]; then
    echo "Generating feature_id on Ubuntu 22.04 ..."
    if [[ ! -x "$SCRIPT_DIR/generator_22" ]]; then
        echo "generator_22 is not executable. Please run: chmod +x $SCRIPT_DIR/generator_22"
        exit 1
    fi
    "$SCRIPT_DIR/generator_22"
else
    echo "Unsupported Ubuntu version: $OS_VERSION"
    exit 1
fi
