#!/bin/bash
set -e

# Resolve the full path of the script
MY_PATH="$( cd "$( dirname "$0" )" && pwd )"

# List of required packages
REQUIRED_PACKAGES=(
    libasio-dev
)

# Function to check if a package is installed
is_installed() {
    dpkg -s "$1" &> /dev/null
}

# Track if we need to update package lists
NEED_UPDATE=false

# Loop through all required packages
for package in "${REQUIRED_PACKAGES[@]}"; do
    if is_installed "$package"; then
        echo "$package is already installed."
    else
        echo "$package is not installed."
        NEED_UPDATE=true
    fi
done

# Install missing packages
if $NEED_UPDATE; then
    echo "Updating package lists..."
    sudo apt update
    echo "Installing missing packages..."
    for package in "${REQUIRED_PACKAGES[@]}"; do
        if ! is_installed "$package"; then
            sudo apt install -y "$package"
        fi
    done
else
    echo "All required packages are already installed. Nothing to do."
fi
