# Camera Setup Scripts for Jetson Orin Nano

Camera streaming and focus control scripts for IMX519 cameras on Jetson Orin Nano.

## Files

| File | Description |
|------|-------------|
| `camera.sh` | Single camera streamer with self-healing |
| `camera-dual.sh` | Dual camera side-by-side streamer |
| `set-camera-focus.py` | Focus control for single or dual cameras |
| `camera-dual.service` | Systemd service for dual camera streaming |

## Quick Start

### Single Camera

```bash
# Stream CAM0 on port 5000
./camera.sh

# Stream CAM1 on port 5001
SENSOR_ID=1 PORT=5001 ./camera.sh

# With custom focus
FOCUS=300 ./camera.sh
```

### Dual Camera (Side-by-Side)

```bash
# Default mode (1080p @ 60fps)
./camera-dual.sh

# 4K mode
./camera-dual.sh --mode 1

# 720p @ 120fps (high speed)
./camera-dual.sh --mode 3

# Show all options
./camera-dual.sh --help
```

### Focus Control

```bash
# Set focus on both cameras
python3 set-camera-focus.py FOCUS=250 --all

# Set focus on specific camera
python3 set-camera-focus.py FOCUS=300 --cam 0
python3 set-camera-focus.py FOCUS=350 --cam 1

# Get current focus
python3 set-camera-focus.py --get FOCUS --all

# Step focus up/down
python3 set-camera-focus.py --step-up FOCUS --all
python3 set-camera-focus.py --step-down FOCUS --all --step 50
```

## Installation

### Install as Systemd Service

```bash
# Copy service file
sudo cp camera-dual.service /etc/systemd/system/

# Enable and start
sudo systemctl daemon-reload
sudo systemctl enable camera-dual.service
sudo systemctl start camera-dual.service

# Check status
sudo systemctl status camera-dual.service
```

### Service Configuration

Edit the service to change settings:

```bash
sudo systemctl edit camera-dual.service
```

Add overrides:
```ini
[Service]
Environment="SENSOR_MODE=1"
Environment="FOCUS=300"
Environment="PORT=5001"
```

## Sensor Modes (IMX519)

| Mode | Resolution | FPS | Use Case |
|------|------------|-----|----------|
| 0 | 4656x3496 | 9 | Full resolution |
| 1 | 3840x2160 | 17 | 4K video |
| 2 | 1920x1080 | 60 | 1080p (default) |
| 3 | 1280x720 | 120 | High-speed |

## Viewing the Stream

```bash
# Using ffplay
ffplay tcp://<jetson-ip>:5000

# Using VLC
vlc tcp://<jetson-ip>:5000

# Using GStreamer
gst-launch-1.0 tcpclientsrc host=<jetson-ip> port=5000 \
  ! tsdemux ! h264parse ! avdec_h264 ! videoconvert ! autovideosink
```

## I2C Bus Mapping

| Camera | CSI Port | I2C Bus | Focus Controller |
|--------|----------|---------|------------------|
| CAM0 | Port 1 | 10 | 0x0C |
| CAM1 | Port 2 | 9 | 0x0C |

## Troubleshooting

### Stream won't start

```bash
# Restart nvargus-daemon
sudo systemctl restart nvargus-daemon

# Check camera detection
ls /dev/video*
sudo dmesg | grep imx519
```

### Focus not working

```bash
# Check I2C devices
sudo i2cdetect -y 9   # CAM1
sudo i2cdetect -y 10  # CAM0

# Test focus manually
python3 set-camera-focus.py FOCUS=500 --cam 0 --verbose
```

### Service keeps restarting

```bash
# Check logs
sudo journalctl -u camera-dual.service -f

# Check for lock file
ls -la /var/lock/camera-dual.lock
sudo rm -f /var/lock/camera-dual.lock
```

## Environment Variables

### camera.sh / camera-dual.sh

| Variable | Default | Description |
|----------|---------|-------------|
| `PORT` | 5000 | TCP port for stream |
| `SENSOR_ID` | 0/1 | Camera to use (camera.sh only) |
| `SENSOR_MODE` | 2 | Sensor mode (0-3) |
| `FOCUS` | 250 | Focus value (0-1000) |
| `BITRATE_KBPS` | 8000 | Encoding bitrate |

### set-camera-focus.py

| Option | Description |
|--------|-------------|
| `--cam 0/1` | Select camera |
| `--all` | Apply to both cameras |
| `--bus N` | I2C bus number |
| `--verbose` | Show debug output |

## License

MIT
