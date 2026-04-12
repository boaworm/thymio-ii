# thymio-ii

Thymio II robot control and experimentation.

## Requirements

- **Python 3.11+**
- **ThymioSuite** (provides the TDM server)
- **Thymio Wireless Dongle** (USB) or direct USB connection

## Installation

```bash
# Create virtual environment
python -m venv .venv
source .venv/bin/activate

# Install dependencies
pip install -r requirements.txt
```

### Dependencies

```
tdmclient>=0.1.21    # Thymio Device Manager client
websockets           # WebSocket transport support
zeroconf             # Robot auto-discovery
```

## Setup

### Launch ThymioSuite

You must run ThymioSuite before using any Python scripts. It provides the TDM server that enables communication with the robot.

```bash
flatpak run org.mobsya.ThymioSuite
```

Keep ThymioSuite running while using the Python scripts. The robot connects via the USB wireless dongle and appears as a network device.

### Connection

By default, scripts use **zeroconf** to auto-discover the robot. No configuration needed.

Optional connection parameters:
- `--addr <IP>` - Manually specify robot IP address
- `--port <PORT>` - Specify TDM port (default: 8596 for TCP)
- `--ws` - Use WebSocket transport instead of TCP (not needed for USB dongle)

**Note:** For USB dongle connections, use the default TCP transport. The `--ws` flag is only needed for remote/networked TDM servers.

## Project Structure

### follow-finger/

Finger-following robot that centers on an object using proximity sensors.

**Usage:**
```bash
python follow-finger/follow.py
```

**Features:**
- Listener-based sensor updates (no polling)
- Automatic reconnection on disconnect
- Configurable detection thresholds

### labyrinth-escaper/

Multiple maze-solving algorithms:

| Script | Algorithm |
|--------|-----------|
| `escaper.py` | Basic wall-following with dead-end recovery |
| `escaper_follow_wall_left.py` | Left-hand rule wall follower |
| `escaper_follow_wall_right.py` | Right-hand rule wall follower |
| `escaper_pledge.py` | Pledge algorithm (tracks rotation) |
| `escaper_tremaux.py` | Tremaux's algorithm (visit mapping) |

**Usage:**
```bash
python labyrinth-escaper/escaper.py
```

**Features:**
- Automatic reconnection on TDM disconnect
- On-board watchdog disabled for stability
- Rear obstacle threshold adjusted (5cm+ ignored)
- Accelerometer-based "victory" detection (tilt to lift robot)

### dashboard/

Real-time sensor display in the terminal.

**Usage:**
```bash
python dashboard/dashboard.py
```

### reset.py

Utility to stop the robot and reset its state.

```bash
python reset.py
```

## Troubleshooting

### Robot not connecting
1. Ensure ThymioSuite is running
2. Check USB dongle is connected
3. Verify robot is powered on (LEDs should be lit)

### Motors not moving
1. Check that nothing is blocking the wheels
2. Ensure robot is lifted off the ground for testing
3. Verify the watchdog is disabled (all scripts now disable it by default)

### Frequent disconnections
1. Use TCP transport (default) instead of WebSocket for USB dongle
2. Ensure no other program is using the TDM connection
3. Keep ThymioSuite in focus

## Sensor Reference

Thymio II horizontal proximity sensors:
```
Front:  [0] [1] [2] [3] [4]  (0=far left, 2=center, 4=far right)
Rear:   [5] [6]              (back sensors)
```

Scale: 0 = clear, ~5000 = touching
