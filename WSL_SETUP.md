# WSL Setup Guide for Arduino Development

Windows driver updates frequently break Arduino USB discovery. This guide sets up development on WSL where USB device access is more stable.

## Why WSL?

- ✅ Arduino IDE/CLI works reliably on WSL
- ✅ USB device discovery stable (no Windows driver interference)
- ✅ Better file I/O performance for development
- ✅ Easy to version control with git
- ✅ Compatible with Cowork mode Claude Code

## Option A: Clone From Windows Folder Into WSL (Recommended)

This syncs your Windows project into WSL while keeping both in sync via git.

### Step 1: Initialize git in Windows folder

Open PowerShell in your project folder:

```powershell
cd "C:\Users\artur\OneDrive\Документи\Claude\Projects\self-balancing-robot"
git init
git add .
git commit -m "Initial Arduino project setup"
```

### Step 2: Create WSL project directory

Open your WSL terminal and run:

```bash
# Create projects directory
mkdir -p ~/Projects
cd ~/Projects

# Copy from Windows OneDrive
cp -r /mnt/c/Users/artur/OneDrive/Документи/Claude/Projects/self-balancing-robot .

# Or use native WSL path if your OneDrive is synced
cd self-balancing-robot
git init  # Initialize git on WSL copy
```

### Step 3: Install arduino-cli on WSL

```bash
sudo apt update
sudo apt install -y arduino-cli

# Verify installation
arduino-cli version

# Install AVR core (for Arduino Uno)
arduino-cli core install arduino:avr
```

### Step 4: Test Arduino discovery

```bash
# Connect Arduino via USB, then:
arduino-cli board list

# Expected output:
# Port         Type              Board Name  FQBN            Core
# /dev/ttyUSB0 Serial Port (USB) Arduino Uno arduino:avr:uno arduino:avr
```

If no Arduino appears, check:
```bash
dmesg | grep -i usb  # See USB device messages
ls -la /dev/tty*     # List available serial ports
```

### Step 5: Compile and Upload

```bash
cd ~/Projects/self-balancing-robot

# Compile sketch
arduino-cli compile --fqbn arduino:avr:uno .

# Upload (replace /dev/ttyUSB0 with your port from 'board list')
arduino-cli upload --fqbn arduino:avr:uno --port /dev/ttyUSB0 .
```

### Step 6: Monitor Serial Output

```bash
# Watch real-time output from robot
arduino-cli monitor --port /dev/ttyUSB0 --config baudrate=115200
```

Press `Ctrl+C` to exit.

---

## Option B: Work Directly in Windows, Upload From WSL

If you prefer editing in Windows but want WSL's better USB support:

### Step 1: Create symlink in WSL

```bash
# In WSL terminal
mkdir -p ~/Projects
ln -s /mnt/c/Users/artur/OneDrive/Документи/Claude/Projects/self-balancing-robot ~/Projects/self-balancing-robot
```

### Step 2: Build and Upload from symlink

```bash
cd ~/Projects/self-balancing-robot

# Compile
arduino-cli compile --fqbn arduino:avr:uno .

# Upload
arduino-cli upload --fqbn arduino:avr:uno --port /dev/ttyUSB0 .
```

**Note**: Windows OneDrive indexing may slow compilation via symlink. Option A (true copy) is faster.

---

## Option C: Git-Based Sync (For teams or multiple machines)

### Windows (source of truth)

```powershell
# Create local git repo
cd "C:\Users\artur\OneDrive\Документи\Claude\Projects\self-balancing-robot"
git init
git add .
git commit -m "Initial commit"

# Push to GitHub/GitLab (optional)
git remote add origin https://github.com/yourusername/self-balancing-robot.git
git push -u origin main
```

### WSL (development)

```bash
# Clone from GitHub or local Windows repo
git clone /mnt/c/Users/artur/OneDrive/Документи/Claude/Projects/self-balancing-robot ~/Projects/self-balancing-robot

# Or clone from GitHub if pushed
git clone https://github.com/yourusername/self-balancing-robot.git ~/Projects/self-balancing-robot

cd ~/Projects/self-balancing-robot
arduino-cli compile --fqbn arduino:avr:uno .
arduino-cli upload --fqbn arduino:avr:uno --port /dev/ttyUSB0 .
```

Sync changes back to Windows:
```bash
# In WSL
git push origin main

# In Windows PowerShell
cd "C:\Users\artur\OneDrive\Документи\Claude\Projects\self-balancing-robot"
git pull
```

---

## Troubleshooting

### Arduino not found in WSL

```bash
# Check if USB device is visible
lsusb | grep Arduino
# or
dmesg | tail -20

# Check permissions (may need udev rules)
sudo usermod -a -G dialout $USER
# Logout and login for changes to take effect
```

### Permission denied on /dev/ttyUSB0

```bash
# Add user to dialout group (permanent fix)
sudo usermod -a -G dialout $USER

# Then logout/login or use:
newgrp dialout
```

### Slow compilation from OneDrive symlink

Use Option A (true copy in WSL) instead of Option B. OneDrive indexing slows cross-filesystem access.

### Can't find arduino-cli command

```bash
# Install from apt
sudo apt install arduino-cli

# Or download binary
curl -fsSL https://raw.githubusercontent.com/arduino/arduino-cli/master/install.sh | DESTDIR=/usr/local/bin sh
```

---

## Recommended Workflow

1. **Edit code** in Windows (VS Code with Arduino extension) or WSL (nano/vim)
2. **Compile & Test** on WSL using `arduino-cli`
3. **Upload** from WSL (stable USB access)
4. **Serial Monitor** on WSL to watch output in real-time
5. **Version control** with git in either OS

This combines the best of both worlds: Windows IDE convenience + WSL reliability.
