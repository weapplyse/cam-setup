# Camera Setup Scripts for Jetson Orin Nano

Camera streaming and focus control scripts for IMX519 cameras on Jetson Orin Nano.

## Files

| File | Description |
|------|-------------|
| `install.sh` | Automated installer (--dual or --single) |
| `camera.sh` | Single camera streamer with self-healing |
| `camera-dual.sh` | Dual camera side-by-side streamer |
| `set-camera-focus.py` | Focus control for single or dual cameras |
| `set-camera-clocks.sh` | Set camera clocks to max (now built into scripts) |
| `camera-config.sh.example` | Example configuration file |

## Installation

### Quick Install

```bash
git clone https://github.com/weapplyse/cam-setup.git
cd cam-setup

# Install dual camera setup (default)
sudo ./install.sh --dual

# Or install single camera setup
sudo ./install.sh --single

# Check installation status
sudo ./install.sh --check

# Uninstall
sudo ./install.sh --uninstall
```

The installer will:
1. Copy scripts to `/home/aspace/`
2. Create config file if not exists
3. Install and enable systemd service
4. Start the camera stream

## Manual Usage

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

## Performance Optimization

Set camera clocks to maximum for best performance:

```bash
sudo ./set-camera-clocks.sh
```

This sets VI, ISP, NVCSI, and EMC clocks to their maximum rates. Run this before starting the camera stream for optimal performance, especially at higher resolutions.

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

## Configuration File

The camera scripts automatically load `/home/aspace/camera-config.sh` if it exists.

Copy the example config and customize:

```bash
cp camera-config.sh.example ~/camera-config.sh
```

Edit `~/camera-config.sh`:

```bash
# Camera configuration - all values are optional
# Scripts use sensible defaults if not specified

# Exposure range (nanoseconds)
EXPOSURE_MIN=34000
EXPOSURE_MAX=100000000

# Gain range
GAIN_MIN=1
GAIN_MAX=12

# Image processing
CONTRAST=1.1
EE_MODE=1           # Edge enhancement mode
EE_STRENGTH=0.8     # Edge enhancement strength

# Encoding
BITRATE=15000000    # Bitrate in bps (will be converted to kbps)
KEYINT=17           # Keyframe interval

# Focus (0-1000)
FOCUS=250

# Video transform
FLIP_METHOD=0       # 0=none, 2=rotate 180

# Resolution (for camera.sh)
SENSOR_MODE=1       # 0-3 (see sensor modes table)
SRC_W=3840          # Source width
SRC_H=2160          # Source height
OUT_W=1920          # Output width (after downscale)
OUT_H=1080          # Output height (after downscale)
```

You can also specify a custom config file:
```bash
CONFIG_FILE=/path/to/my-config.sh ./camera.sh
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
