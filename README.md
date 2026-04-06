# thymo-demo

## Purpose
This project is for experimenting with the Thymio II robot. Each subdirectory contains a robotic program that can be used to control and run the Thymio II robot.

## Requirements

- Python 3.x
- ThymioSuite (for TDM server)
- Thymio Wireless Dongle (USB)

## Setup

### ThymioSuite
You must launch ThymioSuite before running any scripts. It provides the TDM server that enables communication with the robot via the USB dongle.

```bash
flatpak run org.mobsya.ThymioSuite
```

Keep ThymioSuite running while using the Python scripts.

### USB Dongle Configuration
The Thymio Wireless dongle uses the `cdc_ether` kernel module to create a network interface.

```bash
# Load the driver module
sudo modprobe cdc_ether

# The robot should appear at 192.168.1.1
```

## Project Structure

### follow-finger/
Finger-following program for the Thymio II robot. The robot centers on an object (finger) using sensor #2, moves forward when detected, and reverses if too close.

**Usage:**
```bash
# First, ensure ThymioSuite is running
flatpak run org.mobsya.ThymioSuite

# Then run the script
cd follow-finger && python follow.py
```

**Options:**
```bash
python follow.py --addr 192.168.1.1  # Specify robot IP if auto-discovery fails
```

### dashboard/
Real-time sensor dashboard displaying Thymio II sensor data in the terminal.

**Usage:**
```bash
cd dashboard && python dashboard.py
```

### thymio_env/
Thymio II robot environment configuration and setup files.
