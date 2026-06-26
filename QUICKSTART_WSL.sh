#!/bin/bash
# Quick start script for WSL Arduino development
# Copy and paste these commands into your WSL terminal

echo "=== Self-Balancing Robot - WSL Quick Start ==="
echo ""

# Check if arduino-cli is installed
if ! command -v arduino-cli &> /dev/null; then
    echo "Installing arduino-cli..."
    sudo apt update
    sudo apt install -y arduino-cli
fi

# Create project directory in WSL
echo "Setting up project directory..."
mkdir -p ~/Projects
cd ~/Projects

# Copy from Windows (if not already done)
if [ ! -d "self-balancing-robot" ]; then
    echo "Copying project from Windows..."
    cp -r /mnt/c/Users/artur/OneDrive/Документи/Claude/Projects/self-balancing-robot self-balancing-robot
fi

cd self-balancing-robot

# Install AVR board support if not already done
echo "Installing Arduino AVR core..."
arduino-cli core install arduino:avr

echo ""
echo "✓ Setup complete!"
echo ""
echo "Next steps:"
echo "1. Connect Arduino via USB"
echo "2. Find port: arduino-cli board list"
echo "3. Compile: arduino-cli compile --fqbn arduino:avr:uno ."
echo "4. Upload: arduino-cli upload --fqbn arduino:avr:uno --port /dev/ttyUSB0 ."
echo "5. Monitor: arduino-cli monitor --port /dev/ttyUSB0 --config baudrate=115200"
echo ""
