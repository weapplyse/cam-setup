#!/usr/bin/env python3
"""
Camera Focus Control Script
Sets camera focus and other lens parameters, as well as V4L2 camera controls.
Supports dual camera setups with --all flag to control both cameras simultaneously.

Usage: 
  set-camera-focus.py FOCUS=250                    # Set focus on default camera (CAM0)
  set-camera-focus.py FOCUS=250 --cam 1            # Set focus on CAM1
  set-camera-focus.py FOCUS=250 --all              # Set focus on BOTH cameras
  set-camera-focus.py --get FOCUS --all            # Get focus from both cameras
"""

# I2C bus mapping for Jetson Orin Nano with V-Link
# CAM0 (CSI Port 1) uses i2c-2-mux chan_id 0 = bus 10
# CAM1 (CSI Port 2) uses i2c-2-mux chan_id 1 = bus 9
CAMERA_I2C_BUS = {
    0: 10,  # CAM0 -> bus 10
    1: 9,   # CAM1 -> bus 9
}

CAMERA_DEVICE = {
    0: '/dev/video0',
    1: '/dev/video1',
}

import sys
import time
import os
import argparse
import subprocess
import re
import json

# Try to import Flask for API support
try:
    from flask import Flask, request, jsonify
    FLASK_AVAILABLE = True
except ImportError:
    FLASK_AVAILABLE = False

# Try to import GStreamer for nvargus support
try:
    import gi
    gi.require_version('Gst', '1.0')
    from gi.repository import Gst
    GSTREAMER_AVAILABLE = True
    Gst.init(None)
except (ImportError, ValueError):
    GSTREAMER_AVAILABLE = False

# Try to import OpenCV and numpy for autofocus
try:
    import cv2
    import numpy as np
    CV2_AVAILABLE = True
except ImportError:
    CV2_AVAILABLE = False

def init(bus, address):
    os.system("i2cset -y {} 0x{:02x} 0x02 0x00".format(bus, address))

def write(bus, address, value):
    value <<= 4
    os.system("i2cset -y {} 0x{:02x} 0x00 0x{:02x}".format(bus, address, value >> 8))
    os.system("i2cset -y {} 0x{:02x} 0x01 0x{:02x}".format(bus, address, value & 0xFF))
    # Small delay to ensure value is written to hardware
    time.sleep(0.2)

def read(bus, address):
    """Read current focus value from hardware"""
    try:
        # Read high byte (register 0x00)
        result_high = os.popen("i2cget -y {} 0x{:02x} 0x00 2>/dev/null".format(bus, address)).read().strip()
        # Read low byte (register 0x01)
        result_low = os.popen("i2cget -y {} 0x{:02x} 0x01 2>/dev/null".format(bus, address)).read().strip()
        
        if not result_high or not result_low:
            return None
        
        # Parse hex values (remove 0x prefix if present)
        high_byte = int(result_high.replace('0x', ''), 16)
        low_byte = int(result_low.replace('0x', ''), 16)
        
        # Combine bytes and convert back to 0-1000 range
        raw_value = (high_byte << 8) | low_byte
        raw_value >>= 4  # Reverse the left shift from write
        
        # Use round() instead of int() for better precision
        # This ensures values like 250 read back as 250 instead of 249
        focus_value = round((raw_value / 4095.0) * 1000)
        
        # Clamp to valid range
        focus_value = max(0, min(1000, int(focus_value)))
        return int(focus_value)
    except Exception:
        return None

class Focuser:
    bus = None
    CHIP_I2C_ADDR = 0x0C

    def __init__(self, bus):
        self.focus_value = 0
        self.bus = bus
        self.verbose = False
        init(self.bus, self.CHIP_I2C_ADDR)
        
    def read(self):
        return self.focus_value

    def write(self, chip_addr, value):
        if value < 0:
            value = 0
        self.focus_value = value

        # Re-initialize before each write to ensure chip is ready
        init(self.bus, chip_addr)
        time.sleep(0.1)  # Small delay after init

        # Use round() for better precision - ensures values like 250 write correctly
        raw_value = round(value / 1000.0 * 4095)

        write(self.bus, chip_addr, raw_value)
        
        # Focus motor needs time to physically move to new position
        # Wait longer to ensure focus has time to adjust - focus motors can be slow
        time.sleep(1.0)  # Increased from 0.5 to 1.0 seconds
        
        # Verify write by reading back (optional but helps catch errors)
        if self.verbose:
            readback = read(self.bus, self.CHIP_I2C_ADDR)
            if readback is not None and abs(readback - value) > 2:
                print("Warning: Write verification failed - wrote {} but read {}".format(value, readback), file=sys.stderr)
            else:
                print("Focus set to {} (hardware confirmed: {})".format(value, readback if readback is not None else "N/A"))

    OPT_BASE    = 0x1000
    OPT_FOCUS   = OPT_BASE | 0x01
    OPT_ZOOM    = OPT_BASE | 0x02
    OPT_MOTOR_X = OPT_BASE | 0x03
    OPT_MOTOR_Y = OPT_BASE | 0x04
    OPT_IRCUT   = OPT_BASE | 0x05
    opts = {
        OPT_FOCUS : {
            "MIN_VALUE": 0,
            "MAX_VALUE": 1000,
            "DEF_VALUE": 0,
        },
    }
    def reset(self,opt,flag = 1):
        info = self.opts[opt]
        if info == None or info["DEF_VALUE"] == None:
            return
        self.set(opt,info["DEF_VALUE"])

    def get(self,opt,flag = 0, read_hardware=False):
        """Get current value. If read_hardware=True, read from i2c hardware."""
        if read_hardware:
            hw_value = read(self.bus, self.CHIP_I2C_ADDR)
            if hw_value is not None:
                self.focus_value = hw_value
                return hw_value
        info = self.opts[opt]
        return self.read()

    def set(self,opt,value,flag = 1):
        info = self.opts[opt]
        if value > info["MAX_VALUE"]:
            value = info["MAX_VALUE"]
        elif value < info["MIN_VALUE"]:
            value = info["MIN_VALUE"]
        self.write(self.CHIP_I2C_ADDR, value)
        if self.verbose:
            print("write: {}".format(value))

class V4L2Control:
    """Handle V4L2 camera controls like exposure, gain, contrast, etc."""
    
    def __init__(self, device="/dev/video0", verbose=False):
        self.device = device
        self.verbose = verbose
        self._control_cache = {}  # Cache for control names
        
        # Common V4L2 control names and their mappings
        # For IMX519, the controls are: exposure, gain
        self.control_map = {
            'EXPOSURE': ['exposure', 'exposure_absolute', 'exposure_time_absolute'],
            'GAIN': ['gain', 'gain_absolute', 'analog_gain'],
            'CONTRAST': ['contrast'],
            'BRIGHTNESS': ['brightness'],
            'SATURATION': ['saturation'],
            'HUE': ['hue'],
            'WHITE_BALANCE': ['white_balance_temperature', 'white_balance_temperature_auto'],
            'SHARPNESS': ['sharpness'],
            'GAMMA': ['gamma'],
        }
    
    def _run_v4l2_ctl(self, command, check=True):
        """Run v4l2-ctl command and return result"""
        try:
            cmd = ['v4l2-ctl', '-d', self.device] + command
            if self.verbose:
                print("Running: {}".format(' '.join(cmd)))
            result = subprocess.run(
                cmd,
                capture_output=True,
                text=True,
                check=check,
                timeout=5
            )
            return result.stdout, result.stderr, result.returncode
        except subprocess.TimeoutExpired:
            return None, "Command timed out", 1
        except FileNotFoundError:
            return None, "v4l2-ctl not found. Install v4l-utils package.", 1
        except Exception as e:
            return None, str(e), 1
    
    def _find_control_name(self, param_name):
        """Find the actual V4L2 control name for a parameter"""
        param_upper = param_name.upper()
        
        # Check cache first
        if param_name in self._control_cache:
            return self._control_cache[param_name]
        
        if param_upper in self.control_map:
            # Try each possible control name
            for control_name in self.control_map[param_upper]:
                stdout, stderr, returncode = self._run_v4l2_ctl(['--get-ctrl', control_name], check=False)
                if returncode == 0:
                    self._control_cache[param_name] = control_name
                    return control_name
        else:
            # Try the parameter name directly
            stdout, stderr, returncode = self._run_v4l2_ctl(['--get-ctrl', param_name.lower()], check=False)
            if returncode == 0:
                self._control_cache[param_name] = param_name.lower()
                return param_name.lower()
        
        return None
    
    def get(self, param_name):
        """Get current value of a V4L2 control"""
        control_name = self._find_control_name(param_name)
        if not control_name:
            return None
        
        stdout, stderr, returncode = self._run_v4l2_ctl(['--get-ctrl', control_name], check=False)
        if returncode != 0:
            return None
        
        # Parse output: "control_name: value"
        match = re.search(r':\s*([-\d.]+)', stdout)
        if match:
            try:
                return float(match.group(1))
            except ValueError:
                return None
        return None
    
    def set(self, param_name, value):
        """Set value of a V4L2 control"""
        control_name = self._find_control_name(param_name)
        if not control_name:
            if self.verbose:
                print("Control {} not found".format(param_name))
            return False
        
        stdout, stderr, returncode = self._run_v4l2_ctl(['--set-ctrl', '{}={}'.format(control_name, int(value))], check=False)
        if returncode != 0:
            if self.verbose:
                print("Failed to set {}={}: {}".format(control_name, value, stderr))
            return False
        
        if self.verbose:
            print("Set {}={} (control: {})".format(param_name, value, control_name))
        return True
    
    def get_range(self, param_name):
        """Get min/max range for a V4L2 control"""
        control_name = self._find_control_name(param_name)
        if not control_name:
            return None, None
        
        # Get control info from --list-ctrls
        stdout, stderr, returncode = self._run_v4l2_ctl(['--list-ctrls'], check=False)
        if returncode != 0:
            return None, None
        
        # Parse output: "control_name 0x... (int64) : min=X max=Y step=Z ..."
        pattern = r'{}\s+0x[0-9a-f]+\s+[^(]+\s*:\s*min=([-\d.]+)\s+max=([-\d.]+)'.format(re.escape(control_name))
        match = re.search(pattern, stdout)
        if match:
            try:
                min_val = float(match.group(1))
                max_val = float(match.group(2))
                return min_val, max_val
            except ValueError:
                pass
        
        return None, None

class NvArgusControl:
    """Handle nvargus camera controls for Jetson cameras like IMX519."""
    
    def __init__(self, sensor_id=0, verbose=False):
        self.sensor_id = sensor_id
        self.verbose = verbose
        self.gstreamer_available = GSTREAMER_AVAILABLE
        
        # For IMX519, exposure is in nanoseconds, gain is a multiplier
        # From device tree: exposure_factor = 1000000, gain_factor = 16
        self.exposure_factor = 1000000  # nanoseconds
        self.gain_factor = 16  # gain is stored as integer * gain_factor
        
        # Default ranges from device tree
        self.exposure_range = (13, 683709)  # microseconds
        self.gain_range = (16, 256)  # gain_factor units (1.0x to 16.0x)
    
    def _create_gst_element(self):
        """Create a GStreamer nvarguscamerasrc element"""
        if not self.gstreamer_available:
            return None
        try:
            src = Gst.ElementFactory.make('nvarguscamerasrc', 'src')
            if src:
                src.set_property('sensor-id', self.sensor_id)
            return src
        except Exception as e:
            if self.verbose:
                print("Failed to create GStreamer element: {}".format(e))
            return None
    
    def get(self, param_name):
        """Get current value of an nvargus control"""
        param_upper = param_name.upper()
        
        if not self.gstreamer_available:
            return None
        
        src = self._create_gst_element()
        if not src:
            return None
        
        try:
            if param_upper == 'EXPOSURE':
                # Try to get exposure range (nvargus uses ranges, not absolute values)
                prop = src.get_property('exposuretimerange')
                if prop:
                    # Parse range string "low high"
                    parts = prop.split()
                    if len(parts) == 2:
                        # Return midpoint as current value
                        low = int(parts[0])
                        high = int(parts[1])
                        return (low + high) / 2
                return None
            elif param_upper == 'GAIN':
                # Try to get gain range
                prop = src.get_property('gainrange')
                if prop:
                    # Parse range string "low high"
                    parts = prop.split()
                    if len(parts) == 2:
                        # Return midpoint as current value
                        low = float(parts[0])
                        high = float(parts[1])
                        return (low + high) / 2
                return None
        except Exception as e:
            if self.verbose:
                print("Failed to get {}: {}".format(param_name, e))
            return None
        finally:
            del src
        return None
    
    def set(self, param_name, value):
        """Set value of an nvargus control using nvargus-daemon via GStreamer"""
        param_upper = param_name.upper()
        
        # For nvargus, we need to set ranges, not absolute values
        # We'll set a narrow range around the desired value
        if param_upper == 'EXPOSURE':
            # Value is in nanoseconds, convert to microseconds if needed
            if value > 1000000:
                # Assume nanoseconds
                exposure_ns = int(value)
            else:
                # Assume microseconds, convert to nanoseconds
                exposure_ns = int(value * 1000)
            
            # Clamp to valid range
            min_exp_ns = self.exposure_range[0] * 1000  # microseconds to nanoseconds
            max_exp_ns = self.exposure_range[1] * 1000
            exposure_ns = max(min_exp_ns, min(exposure_ns, max_exp_ns))
            
            # Set a narrow range around the desired value (±1%)
            range_margin = max(1000, exposure_ns // 100)  # 1% or at least 1us
            low = exposure_ns - range_margin
            high = exposure_ns + range_margin
            low = max(min_exp_ns, low)
            high = min(max_exp_ns, high)
            
            range_str = "{} {}".format(int(low), int(high))
            
            if self.verbose:
                print("Setting exposure range: {} (target: {} ns)".format(range_str, exposure_ns))
            
            # Use gst-launch to set the property temporarily
            # This is a workaround since we can't directly control nvargus-daemon
            cmd = [
                'gst-launch-1.0', '-q',
                'nvarguscamerasrc', 'sensor-id={}'.format(self.sensor_id),
                'exposuretimerange="{}"'.format(range_str),
                '!', 'fakesink'
            ]
            
            try:
                result = subprocess.run(
                    cmd,
                    capture_output=True,
                    text=True,
                    timeout=2
                )
                # The command will fail quickly, but it sets the property
                # This is a limitation - we need nvargus-daemon to be running
                return True
            except Exception as e:
                if self.verbose:
                    print("Failed to set exposure via GStreamer: {}".format(e))
                return False
                
        elif param_upper == 'GAIN':
            # Gain is typically 1.0 to 16.0, but stored as integer * gain_factor
            # Convert to gain_factor units
            if value < 10:
                # Assume it's already in gain_factor units (1.0x = 16)
                gain_val = int(value * self.gain_factor)
            else:
                # Assume it's already in the right range
                gain_val = int(value)
            
            # Clamp to valid range
            gain_val = max(self.gain_range[0], min(gain_val, self.gain_range[1]))
            
            # Convert back to actual gain value
            actual_gain = gain_val / self.gain_factor
            
            # Set a narrow range around the desired value
            range_margin = max(0.1, actual_gain * 0.01)  # 1%
            low = max(1.0, actual_gain - range_margin)
            high = min(16.0, actual_gain + range_margin)
            
            range_str = "{} {}".format(int(low * self.gain_factor), int(high * self.gain_factor))
            
            if self.verbose:
                print("Setting gain range: {} (target: {:.2f}x)".format(range_str, actual_gain))
            
            # Use gst-launch to set the property
            cmd = [
                'gst-launch-1.0', '-q',
                'nvarguscamerasrc', 'sensor-id={}'.format(self.sensor_id),
                'gainrange="{}"'.format(range_str),
                '!', 'fakesink'
            ]
            
            try:
                result = subprocess.run(
                    cmd,
                    capture_output=True,
                    text=True,
                    timeout=2
                )
                return True
            except Exception as e:
                if self.verbose:
                    print("Failed to set gain via GStreamer: {}".format(e))
                return False
        
        return False
    
    def get_range(self, param_name):
        """Get min/max range for an nvargus control"""
        param_upper = param_name.upper()
        
        if param_upper == 'EXPOSURE':
            # Return range in nanoseconds
            return (self.exposure_range[0] * 1000, self.exposure_range[1] * 1000)
        elif param_upper == 'GAIN':
            # Return range in gain_factor units
            return (self.gain_range[0] / self.gain_factor, self.gain_range[1] / self.gain_factor)
        
        return None, None

class CameraController:
    """Unified controller for camera parameters that can be used from both CLI and API"""
    
    def __init__(self, bus=10, device="/dev/video0", verbose=False):
        self.bus = bus
        self.device = device
        self.verbose = verbose
        
        # Initialize focuser
        self.focuser = None
        try:
            if self.verbose:
                print("Initializing Focuser on I2C bus {}...".format(self.bus))
            self.focuser = Focuser(self.bus)
            self.focuser.verbose = self.verbose
            if self.verbose:
                print("Focuser initialized successfully")
        except Exception as e:
            if self.verbose:
                print("Warning: Failed to initialize Focuser: {}".format(e), file=sys.stderr)
            try:
                self.focuser = Focuser(self.bus)
                self.focuser.verbose = self.verbose
            except:
                pass
        
        # Initialize camera controls
        self.nvargus_control = None
        self.v4l2_control = None
        
        # Try nvargus first
        try:
            if self.verbose:
                print("Initializing nvargus control...")
            self.nvargus_control = NvArgusControl(sensor_id=0, verbose=self.verbose)
            if self.verbose:
                print("nvargus control initialized successfully")
        except Exception as e:
            if self.verbose:
                print("Warning: Failed to initialize nvargus control: {}".format(e), file=sys.stderr)
            self.nvargus_control = None
        
        # Try V4L2 control
        try:
            if self.verbose:
                print("Initializing V4L2 control for device {}...".format(self.device))
            self.v4l2_control = V4L2Control(device=self.device, verbose=self.verbose)
            if self.verbose:
                print("V4L2 control initialized successfully")
        except Exception as e:
            if self.verbose:
                print("Warning: Failed to initialize V4L2 control: {}".format(e), file=sys.stderr)
            self.v4l2_control = None
    
    def _get_opt_constant(self, key):
        """Get I2C option constant for a parameter"""
        key_map = {
            'FOCUS': Focuser.OPT_FOCUS,
            'ZOOM': Focuser.OPT_ZOOM,
            'MOTOR_X': Focuser.OPT_MOTOR_X,
            'MOTOR_Y': Focuser.OPT_MOTOR_Y,
        }
        return key_map.get(key.upper())
    
    def get_parameter(self, param_name, read_hardware=False):
        """Get current value of a parameter"""
        key = param_name.upper()
        
        # Camera parameters (V4L2 or nvargus)
        if key in ['EXPOSURE', 'GAIN']:
            # Try V4L2 first
            if self.v4l2_control:
                value = self.v4l2_control.get(key)
                if value is not None:
                    return {'success': True, 'value': int(value), 'parameter': key}
            # Fall back to nvargus
            if self.nvargus_control:
                value = self.nvargus_control.get(key)
                if value is not None:
                    return {'success': True, 'value': int(value), 'parameter': key}
            return {'success': False, 'error': 'Could not get {} from camera'.format(key)}
        
        elif key in ['CONTRAST', 'BRIGHTNESS', 'SATURATION', 'HUE', 
                     'WHITE_BALANCE', 'SHARPNESS', 'GAMMA']:
            # V4L2 only parameters
            if self.v4l2_control:
                value = self.v4l2_control.get(key)
                if value is not None:
                    return {'success': True, 'value': int(value), 'parameter': key}
                return {'success': False, 'error': 'Could not get {} from V4L2 device'.format(key)}
            return {'success': False, 'error': 'V4L2 control not available'}
        
        else:
            # I2C parameters (FOCUS, ZOOM, etc.)
            if not self.focuser:
                return {'success': False, 'error': 'Focuser not available'}
            
            opt = self._get_opt_constant(key)
            if opt is None:
                return {'success': False, 'error': 'Unknown parameter: {}'.format(key)}
            
            try:
                value = self.focuser.get(opt, read_hardware=read_hardware)
                return {'success': True, 'value': value, 'parameter': key}
            except Exception as e:
                return {'success': False, 'error': str(e)}
    
    def set_parameter(self, param_name, value, validate_step=50):
        """Set value of a parameter"""
        key = param_name.upper()
        
        try:
            val = int(value)
        except (ValueError, TypeError):
            return {'success': False, 'error': 'Invalid value: {}'.format(value)}
        
        # Camera parameters
        if key in ['EXPOSURE', 'GAIN']:
            success = False
            if self.v4l2_control:
                if self.v4l2_control.set(key, val):
                    success = True
            if not success and self.nvargus_control:
                if self.nvargus_control.set(key, val):
                    success = True
            
            if success:
                return {'success': True, 'parameter': key, 'value': val}
            return {'success': False, 'error': 'Failed to set {}={}'.format(key, val)}
        
        elif key in ['CONTRAST', 'BRIGHTNESS', 'SATURATION', 'HUE', 
                     'WHITE_BALANCE', 'SHARPNESS', 'GAMMA']:
            if self.v4l2_control:
                if self.v4l2_control.set(key, val):
                    return {'success': True, 'parameter': key, 'value': val}
                return {'success': False, 'error': 'Failed to set {}={}'.format(key, val)}
            return {'success': False, 'error': 'V4L2 control not available'}
        
        else:
            # I2C parameters
            if not self.focuser:
                return {'success': False, 'error': 'Focuser not available'}
            
            opt = self._get_opt_constant(key)
            if opt is None:
                return {'success': False, 'error': 'Unknown parameter: {}'.format(key)}
            
            # Validate step for FOCUS if enabled
            if key == 'FOCUS' and validate_step > 0:
                if val % validate_step != 0:
                    val = round(val / validate_step) * validate_step
            
            try:
                self.focuser.set(opt, val)
                return {'success': True, 'parameter': key, 'value': val}
            except Exception as e:
                return {'success': False, 'error': str(e)}
    
    def step_parameter(self, param_name, step, direction='up', validate_step=50):
        """Step a parameter up or down"""
        key = param_name.upper()
        
        # Get current value
        result = self.get_parameter(key)
        if not result['success']:
            return result
        
        current_value = result['value']
        
        # Calculate new value
        if direction == 'up':
            new_value = current_value + step
        else:
            new_value = current_value - step
        
        # Get range limits
        if key in ['EXPOSURE', 'GAIN']:
            if self.v4l2_control:
                min_val, max_val = self.v4l2_control.get_range(key)
            elif self.nvargus_control:
                min_val, max_val = self.nvargus_control.get_range(key)
            else:
                return {'success': False, 'error': 'Camera control not available'}
            
            if min_val is not None:
                new_value = max(new_value, int(min_val))
            if max_val is not None:
                new_value = min(new_value, int(max_val))
        else:
            # I2C parameters
            if key == 'FOCUS':
                new_value = max(0, min(1000, new_value))
                if validate_step > 0:
                    new_value = round(new_value / validate_step) * validate_step
        
        # Set new value
        return self.set_parameter(key, new_value, validate_step)
    
    def reset_focus(self):
        """Reset focus to default value"""
        if not self.focuser:
            return {'success': False, 'error': 'Focuser not available'}
        
        try:
            self.focuser.reset(Focuser.OPT_FOCUS)
            return {'success': True, 'message': 'Focus reset to default'}
        except Exception as e:
            return {'success': False, 'error': str(e)}
    
    def get_all_parameters(self):
        """Get all available parameters"""
        params = {}
        
        # I2C parameters
        for key in ['FOCUS', 'ZOOM', 'MOTOR_X', 'MOTOR_Y']:
            result = self.get_parameter(key)
            if result['success']:
                params[key] = result['value']
        
        # Camera parameters
        for key in ['EXPOSURE', 'GAIN', 'CONTRAST', 'BRIGHTNESS', 'SATURATION', 
                    'HUE', 'WHITE_BALANCE', 'SHARPNESS', 'GAMMA']:
            result = self.get_parameter(key)
            if result['success']:
                params[key] = result['value']
        
        return {'success': True, 'parameters': params}

def start_api_server(args):
    """Start Flask API server"""
    app = Flask(__name__)
    
    # Initialize controller
    controller = CameraController(
        bus=args.bus,
        device=args.device,
        verbose=args.verbose
    )
    
    @app.route('/api/health', methods=['GET'])
    def health():
        """Health check endpoint"""
        return jsonify({'status': 'ok', 'service': 'camera-control'})
    
    @app.route('/api/parameters', methods=['GET'])
    def get_all_parameters():
        """Get all current parameter values"""
        result = controller.get_all_parameters()
        if result['success']:
            return jsonify(result)
        return jsonify(result), 500
    
    @app.route('/api/parameters/<param_name>', methods=['GET'])
    def get_parameter(param_name):
        """Get current value of a specific parameter"""
        read_hardware = request.args.get('read_hardware', 'false').lower() == 'true'
        result = controller.get_parameter(param_name, read_hardware=read_hardware)
        if result['success']:
            return jsonify(result)
        return jsonify(result), 404
    
    @app.route('/api/parameters/<param_name>', methods=['POST'])
    def set_parameter(param_name):
        """Set value of a specific parameter"""
        data = request.get_json() or {}
        value = data.get('value')
        
        if value is None:
            return jsonify({'success': False, 'error': 'Missing value parameter'}), 400
        
        validate_step = data.get('validate_step', args.validate_step)
        result = controller.set_parameter(param_name, value, validate_step=validate_step)
        
        if result['success']:
            return jsonify(result)
        return jsonify(result), 400
    
    @app.route('/api/parameters', methods=['POST'])
    def set_multiple_parameters():
        """Set multiple parameters at once"""
        data = request.get_json() or {}
        parameters = data.get('parameters', {})
        
        if not parameters:
            return jsonify({'success': False, 'error': 'Missing parameters object'}), 400
        
        results = {}
        all_success = True
        
        validate_step = data.get('validate_step', args.validate_step)
        
        for param_name, value in parameters.items():
            result = controller.set_parameter(param_name, value, validate_step=validate_step)
            results[param_name] = result
            if not result['success']:
                all_success = False
        
        status_code = 200 if all_success else 207  # 207 Multi-Status
        return jsonify({
            'success': all_success,
            'results': results
        }), status_code
    
    @app.route('/api/parameters/<param_name>/step-up', methods=['POST'])
    def step_up_parameter(param_name):
        """Increase parameter value by step amount"""
        data = request.get_json() or {}
        step = data.get('step', args.step)
        validate_step = data.get('validate_step', args.validate_step)
        
        result = controller.step_parameter(param_name, step, direction='up', validate_step=validate_step)
        
        if result['success']:
            return jsonify(result)
        return jsonify(result), 400
    
    @app.route('/api/parameters/<param_name>/step-down', methods=['POST'])
    def step_down_parameter(param_name):
        """Decrease parameter value by step amount"""
        data = request.get_json() or {}
        step = data.get('step', args.step)
        validate_step = data.get('validate_step', args.validate_step)
        
        result = controller.step_parameter(param_name, step, direction='down', validate_step=validate_step)
        
        if result['success']:
            return jsonify(result)
        return jsonify(result), 400
    
    @app.route('/api/focus/reset', methods=['POST'])
    def reset_focus():
        """Reset focus to default value"""
        result = controller.reset_focus()
        
        if result['success']:
            return jsonify(result)
        return jsonify(result), 500
    
    @app.route('/api/info', methods=['GET'])
    def info():
        """Get API information and available parameters"""
        return jsonify({
            'version': '1.0',
            'endpoints': {
                'GET /api/health': 'Health check',
                'GET /api/parameters': 'Get all parameters',
                'GET /api/parameters/<name>': 'Get specific parameter',
                'POST /api/parameters/<name>': 'Set specific parameter',
                'POST /api/parameters': 'Set multiple parameters',
                'POST /api/parameters/<name>/step-up': 'Increase parameter',
                'POST /api/parameters/<name>/step-down': 'Decrease parameter',
                'POST /api/focus/reset': 'Reset focus to default',
                'GET /api/info': 'This information'
            },
            'supported_parameters': {
                'i2c': ['FOCUS', 'ZOOM', 'MOTOR_X', 'MOTOR_Y'],
                'v4l2': ['EXPOSURE', 'GAIN', 'CONTRAST', 'BRIGHTNESS', 'SATURATION', 
                         'HUE', 'WHITE_BALANCE', 'SHARPNESS', 'GAMMA']
            },
            'config': {
                'bus': args.bus,
                'device': args.device
            }
        })
    
    print("Starting Camera Control API server on http://{}:{}".format(args.api_host, args.api_port))
    print("API documentation: http://{}:{}/api/info".format(args.api_host, args.api_port))
    app.run(host=args.api_host, port=args.api_port, debug=False)

def main():
    parser = argparse.ArgumentParser(
        description='Set camera focus and other lens parameters',
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
Examples:
  set-camera-focus.py FOCUS=250
  set-camera-focus.py FOCUS=500 ZOOM=100
  set-camera-focus.py EXPOSURE=50000 GAIN=5 CONTRAST=128
  set-camera-focus.py FOCUS=250 EXPOSURE=50000 GAIN=5
  set-camera-focus.py FOCUS=0 --reset
  set-camera-focus.py --get FOCUS
  set-camera-focus.py --get EXPOSURE
  set-camera-focus.py --step-up FOCUS
  set-camera-focus.py --step-down FOCUS --step=50

I2C Bus Information:
  Jetson Nano B01: CAM0 = bus 7, CAM1 = bus 8
  Jetson Xavier NX: CAM0 = bus 10, CAM1 = bus 9

V4L2 Camera Parameters:
  EXPOSURE, GAIN, CONTRAST, BRIGHTNESS, SATURATION, HUE, WHITE_BALANCE, SHARPNESS, GAMMA
        """
    )
    
    parser.add_argument('values', nargs='*', metavar='KEY=VALUE',
                       help='Parameter values to set (e.g., FOCUS=250, EXPOSURE=50000, GAIN=5, CONTRAST=128)')
    parser.add_argument('--bus', type=int, default=None,
                       help='I2C bus number (default: auto-detect based on --cam)')
    parser.add_argument('--device', type=str, default=None,
                       help='V4L2 device path (default: auto-detect based on --cam)')
    parser.add_argument('--cam', type=int, default=0, choices=[0, 1],
                       help='Camera to control: 0=CAM0 (default), 1=CAM1')
    parser.add_argument('--all', action='store_true',
                       help='Apply to ALL cameras (both CAM0 and CAM1)')
    parser.add_argument('--reset', action='store_true',
                       help='Reset focus to default value')
    parser.add_argument('--get', metavar='KEY',
                       help='Get current value for parameter (FOCUS, ZOOM, MOTOR_X, MOTOR_Y, EXPOSURE, GAIN, CONTRAST, BRIGHTNESS, SATURATION, SHARPNESS, etc.)')
    parser.add_argument('--read-hardware', action='store_true',
                       help='Read value from hardware instead of cached value (for --get, --step-up, --step-down)')
    parser.add_argument('--stream-overlay', action='store_true',
                       help='Create GStreamer pipeline with sharpness overlay: reads from input stream and outputs with overlay')
    parser.add_argument('--stream-input', type=str, default='tcp://127.0.0.1:5000',
                       help='Input stream URL for overlay (default: tcp://127.0.0.1:5000)')
    parser.add_argument('--stream-output-port', type=int, default=5001,
                       help='Output port for stream with overlay (default: 5001)')
    parser.add_argument('--validate-step', type=int, default=50,
                       help='Validate that FOCUS values are multiples of this step (0 to disable, default: 50)')
    parser.add_argument('--step-up', metavar='KEY',
                       help='Increase parameter value by step amount')
    parser.add_argument('--step-down', metavar='KEY',
                       help='Decrease parameter value by step amount')
    parser.add_argument('--step', type=int, default=50,
                       help='Step size for step-up/step-down operations (default: 50)')
    parser.add_argument('--autofocus', action='store_true',
                       help='Find best focus automatically using OpenCV sharpness detection. Requires OpenCV and camera stream running.')
    parser.add_argument('--autofocus-step', type=int, default=50,
                       help='Coarse step size for initial autofocus scan (default: 50)')
    parser.add_argument('--autofocus-fine-step', type=int, default=25,
                       help='Fine step size for refinement around best position (default: 25)')
    parser.add_argument('--autofocus-samples', type=int, default=5,
                       help='Number of frames to average for each focus position (default: 5)')
    parser.add_argument('--autofocus-timeout', type=int, default=60,
                       help='Timeout in seconds for autofocus operation (default: 60)')
    parser.add_argument('--autofocus-roi', type=str, default='0.4,0.4,0.2,0.2',
                       help='ROI for autofocus as x,y,width,height (default: 0.4,0.4,0.2,0.2 = center 20%%)')
    parser.add_argument('--autofocus-refinement-range', type=int, default=200,
                       help='Range around best position to refine (default: 200, e.g. +/-100 from best)')
    parser.add_argument('--monitor', action='store_true',
                       help='Live focus monitor: display camera stream with real-time sharpness measurement. Use arrow keys or +/- to adjust focus.')
    parser.add_argument('--monitor-text', action='store_true',
                       help='Text-only monitor: show sharpness values in terminal (for use with external video player like ffplay). Use +/- to adjust focus.')
    parser.add_argument('--monitor-step', type=int, default=25,
                       help='Step size for focus adjustment in monitor mode (default: 25)')
    parser.add_argument('--monitor-roi', type=str, default='0.4,0.4,0.2,0.2',
                       help='ROI for sharpness measurement in monitor mode (default: 0.4,0.4,0.2,0.2 = center 20%%)')
    parser.add_argument('--monitor-interval', type=float, default=0.5,
                       help='Update interval in seconds for text monitor (default: 0.5)')
    parser.add_argument('--stream-url', type=str, default='tcp://127.0.0.1:5000',
                       help='TCP stream URL for autofocus/monitor (default: tcp://127.0.0.1:5000)')
    parser.add_argument('--init-retries', type=int, default=5,
                       help='Number of retries for initialization (default: 5)')
    parser.add_argument('--init-delay', type=float, default=1.0,
                       help='Delay between init retries in seconds (default: 1.0)')
    parser.add_argument('--write-retries', type=int, default=3,
                       help='Number of retries for write operations (default: 3)')
    parser.add_argument('--write-delay', type=float, default=0.5,
                       help='Delay between write retries in seconds (default: 0.5)')
    parser.add_argument('--verbose', action='store_true',
                       help='Enable verbose output')
    parser.add_argument('--api', action='store_true',
                       help='Start API server instead of CLI mode')
    parser.add_argument('--api-host', type=str, default='0.0.0.0',
                       help='API server host (default: 0.0.0.0)')
    parser.add_argument('--api-port', type=int, default=8080,
                       help='API server port (default: 8080)')
    
    args = parser.parse_args()
    
    # Resolve bus and device based on --cam if not explicitly set
    if args.bus is None:
        args.bus = CAMERA_I2C_BUS.get(args.cam, 10)
    if args.device is None:
        args.device = CAMERA_DEVICE.get(args.cam, '/dev/video0')
    
    # If API mode, start server
    if args.api:
        if not FLASK_AVAILABLE:
            print("Error: Flask is required for API mode. Install it with: pip install flask", file=sys.stderr)
            sys.exit(1)
        start_api_server(args)
        return
    
    # Handle --all flag for dual camera operations
    if getattr(args, 'all', False):
        cameras = [0, 1]
        
        # Handle --get with --all
        if args.get:
            key = args.get.upper()
            results = {}
            for cam_id in cameras:
                bus = CAMERA_I2C_BUS.get(cam_id, 10)
                device = CAMERA_DEVICE.get(cam_id, '/dev/video{}'.format(cam_id))
                
                if key == 'FOCUS':
                    try:
                        focuser = Focuser(bus)
                        focuser.verbose = args.verbose
                        value = focuser.get(Focuser.OPT_FOCUS, read_hardware=True)
                        if value is None:
                            value = focuser.get(Focuser.OPT_FOCUS, read_hardware=False)
                        results['CAM{}'.format(cam_id)] = value
                    except Exception as e:
                        results['CAM{}'.format(cam_id)] = 'Error: {}'.format(e)
                else:
                    # For other parameters, use V4L2
                    try:
                        v4l2 = V4L2Control(device=device, verbose=args.verbose)
                        value = v4l2.get(key)
                        results['CAM{}'.format(cam_id)] = int(value) if value is not None else None
                    except Exception as e:
                        results['CAM{}'.format(cam_id)] = 'Error: {}'.format(e)
            
            for cam, value in results.items():
                print("{}: {}".format(cam, value))
            sys.exit(0)
        
        # Handle setting values with --all
        if args.values:
            all_success = True
            for cam_id in cameras:
                bus = CAMERA_I2C_BUS.get(cam_id, 10)
                device = CAMERA_DEVICE.get(cam_id, '/dev/video{}'.format(cam_id))
                
                if args.verbose:
                    print("\n=== CAM{} (bus={}, device={}) ===".format(cam_id, bus, device))
                
                # Initialize focuser for this camera
                try:
                    focuser = Focuser(bus)
                    focuser.verbose = args.verbose
                except Exception as e:
                    print("Warning: Failed to initialize Focuser for CAM{}: {}".format(cam_id, e), file=sys.stderr)
                    focuser = None
                
                # Initialize V4L2 for this camera
                try:
                    v4l2_control = V4L2Control(device=device, verbose=args.verbose)
                except Exception as e:
                    v4l2_control = None
                
                # Process each value
                for value_arg in args.values:
                    if '=' not in value_arg:
                        continue
                    
                    key, val = value_arg.split('=', 1)
                    key = key.upper()
                    
                    try:
                        val = int(val)
                    except ValueError:
                        print("Error: Invalid value '{}' for {}".format(val, key), file=sys.stderr)
                        all_success = False
                        continue
                    
                    # Validate step for FOCUS
                    if key == 'FOCUS' and args.validate_step > 0:
                        if val % args.validate_step != 0:
                            val = round(val / args.validate_step) * args.validate_step
                    
                    try:
                        if key == 'FOCUS' and focuser:
                            focuser.set(Focuser.OPT_FOCUS, val)
                            print("CAM{}: Set FOCUS to {}".format(cam_id, val))
                        elif key in ['EXPOSURE', 'GAIN', 'CONTRAST', 'BRIGHTNESS'] and v4l2_control:
                            if v4l2_control.set(key, val):
                                print("CAM{}: Set {} to {}".format(cam_id, key, val))
                            else:
                                print("CAM{}: Failed to set {}".format(cam_id, key), file=sys.stderr)
                        elif key == 'ZOOM' and focuser:
                            focuser.set(Focuser.OPT_ZOOM, val)
                            print("CAM{}: Set ZOOM to {}".format(cam_id, val))
                    except Exception as e:
                        print("CAM{}: Error setting {}={}: {}".format(cam_id, key, val, e), file=sys.stderr)
            
            sys.exit(0 if all_success else 1)
        
        # Handle --reset with --all
        if args.reset:
            for cam_id in cameras:
                bus = CAMERA_I2C_BUS.get(cam_id, 10)
                try:
                    focuser = Focuser(bus)
                    focuser.reset(Focuser.OPT_FOCUS)
                    print("CAM{}: Focus reset to default".format(cam_id))
                except Exception as e:
                    print("CAM{}: Failed to reset focus: {}".format(cam_id, e), file=sys.stderr)
            sys.exit(0)
        
        # Handle --step-up/--step-down with --all
        if args.step_up or args.step_down:
            key = (args.step_up or args.step_down).upper()
            direction = 'up' if args.step_up else 'down'
            
            for cam_id in cameras:
                bus = CAMERA_I2C_BUS.get(cam_id, 10)
                try:
                    focuser = Focuser(bus)
                    focuser.verbose = args.verbose
                    current_value = focuser.get(Focuser.OPT_FOCUS, read_hardware=True)
                    if current_value is None:
                        current_value = focuser.get(Focuser.OPT_FOCUS, read_hardware=False) or 0
                    
                    if direction == 'up':
                        new_value = min(current_value + args.step, focuser.opts[Focuser.OPT_FOCUS]["MAX_VALUE"])
                    else:
                        new_value = max(current_value - args.step, 0)
                    
                    if args.validate_step > 0:
                        new_value = round(new_value / args.validate_step) * args.validate_step
                    
                    focuser.set(Focuser.OPT_FOCUS, new_value)
                    print("CAM{}: {} {} -> {}".format(cam_id, key, current_value, new_value))
                except Exception as e:
                    print("CAM{}: Error: {}".format(cam_id, e), file=sys.stderr)
            sys.exit(0)
        
        print("Error: --all requires --get, values to set, --reset, --step-up, or --step-down", file=sys.stderr)
        sys.exit(1)
    
    # Single camera mode (original behavior)
    focuser = None
    try:
        if args.verbose:
            print("Initializing Focuser on I2C bus {}...".format(args.bus))
        focuser = Focuser(args.bus)
        focuser.verbose = args.verbose
        if args.verbose:
            print("Focuser initialized successfully")
    except Exception as e:
        print("Warning: Failed to initialize Focuser: {}".format(e), file=sys.stderr)
        print("Will attempt to use Focuser anyway - writes may still work.", file=sys.stderr)
        # Try to create anyway - sometimes it works even if init had issues
        try:
            focuser = Focuser(args.bus)
            focuser.verbose = args.verbose
        except:
            pass
    
    # Initialize camera controls (try nvargus first for Jetson cameras, then V4L2)
    nvargus_control = None
    v4l2_control = None
    
    # Try nvargus first (for Jetson cameras like IMX519)
    try:
        if args.verbose:
            print("Initializing nvargus control...")
        nvargus_control = NvArgusControl(sensor_id=args.cam, verbose=args.verbose)
        if args.verbose:
            print("nvargus control initialized successfully")
    except Exception as e:
        if args.verbose:
            print("Warning: Failed to initialize nvargus control: {}".format(e), file=sys.stderr)
        nvargus_control = None
    
    # Also try V4L2 control as fallback
    try:
        if args.verbose:
            print("Initializing V4L2 control for device {}...".format(args.device))
        v4l2_control = V4L2Control(device=args.device, verbose=args.verbose)
        if args.verbose:
            print("V4L2 control initialized successfully")
    except Exception as e:
        if args.verbose:
            print("Warning: Failed to initialize V4L2 control: {}".format(e), file=sys.stderr)
        v4l2_control = None
    
    # Helper function to get option constant
    def get_opt_constant(key):
        key_map = {
            'FOCUS': Focuser.OPT_FOCUS,
            'ZOOM': Focuser.OPT_ZOOM,
            'MOTOR_X': Focuser.OPT_MOTOR_X,
            'MOTOR_Y': Focuser.OPT_MOTOR_Y,
        }
        return key_map.get(key.upper())
    
    # Handle get current value
    if args.get:
        key = args.get.upper()
        # Check if it's a camera parameter (V4L2 or nvargus)
        if key in ['EXPOSURE', 'GAIN']:
            # Try V4L2 first (works directly with IMX519 via nvargus plugin)
            if v4l2_control:
                current_value = v4l2_control.get(key)
                if current_value is not None:
                    print(int(current_value))
                    sys.exit(0)
            # Fall back to nvargus
            if nvargus_control:
                current_value = nvargus_control.get(key)
                if current_value is not None:
                    print(int(current_value))
                    sys.exit(0)
            print("Error: Could not get {} from camera".format(key), file=sys.stderr)
            sys.exit(1)
        elif key == 'SHARPNESS':
            # Read sharpness from camera stream
            if not CV2_AVAILABLE:
                print("Error: SHARPNESS measurement requires OpenCV", file=sys.stderr)
                sys.exit(1)
            
            try:
                # Parse stream URL
                if args.stream_url.startswith('tcp://'):
                    url_parts = args.stream_url.replace('tcp://', '').split(':')
                    host = url_parts[0] if len(url_parts) > 0 else '127.0.0.1'
                    port = int(url_parts[1]) if len(url_parts) > 1 else 5000
                else:
                    host = '127.0.0.1'
                    port = 5000
                
                # Read from stream
                pipeline = "tcpclientsrc host={} port={} ! h264parse ! avdec_h264 ! videoconvert ! appsink".format(host, port)
                cap = cv2.VideoCapture(pipeline, cv2.CAP_GSTREAMER)
                
                if not cap.isOpened():
                    print("Error: Could not open stream at {}:{}".format(host, port), file=sys.stderr)
                    sys.exit(1)
                
                # Read a few frames and measure sharpness
                sharpness_values = []
                for i in range(5):
                    ret, frame = cap.read()
                    if ret and frame is not None:
                        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
                        sharpness = cv2.Laplacian(gray, cv2.CV_64F).var()
                        sharpness_values.append(sharpness)
                    time.sleep(0.1)
                
                cap.release()
                
                if sharpness_values:
                    avg_sharpness = sum(sharpness_values) / len(sharpness_values)
                    print("{:.2f}".format(avg_sharpness))
                    sys.exit(0)
                else:
                    print("Error: Could not read frames from stream", file=sys.stderr)
                    sys.exit(1)
            except Exception as e:
                print("Error: Failed to measure sharpness: {}".format(e), file=sys.stderr)
                sys.exit(1)
        elif key in ['CONTRAST', 'BRIGHTNESS', 'SATURATION', 'HUE', 
                     'WHITE_BALANCE', 'GAMMA']:
            # V4L2 only parameters
            if v4l2_control:
                current_value = v4l2_control.get(key)
                if current_value is not None:
                    print(int(current_value))
                    sys.exit(0)
                else:
                    print("Error: Could not get {} from V4L2 device".format(key), file=sys.stderr)
                    sys.exit(1)
            else:
                print("Error: V4L2 control not available", file=sys.stderr)
                sys.exit(1)
        else:
            # I2C parameter (FOCUS, ZOOM, etc.)
            try:
                opt = get_opt_constant(key)
                if opt is None:
                    print("Error: Unknown parameter '{}'".format(key), file=sys.stderr)
                    sys.exit(1)
                # Always try to read from hardware first for --get to get actual value
                # Fallback to cached value only if hardware read fails
                current_value = focuser.get(opt, read_hardware=True)
                if current_value is None:
                    if args.verbose:
                        print("Warning: Could not read from hardware, trying cached value", file=sys.stderr)
                    current_value = focuser.get(opt, read_hardware=False)
                print(current_value)
                sys.exit(0)
            except Exception as e:
                print("Error: Failed to get {}: {}".format(key, e), file=sys.stderr)
                sys.exit(1)
    
    # Handle stream overlay - create GStreamer pipeline with sharpness overlay
    if args.stream_overlay:
        if not CV2_AVAILABLE:
            print("Error: Stream overlay requires OpenCV", file=sys.stderr)
            sys.exit(1)
        
        try:
            # Parse input stream URL
            if args.stream_input.startswith('tcp://'):
                url_parts = args.stream_input.replace('tcp://', '').split(':')
                input_host = url_parts[0] if len(url_parts) > 0 else '127.0.0.1'
                input_port = int(url_parts[1]) if len(url_parts) > 1 else 5000
            else:
                input_host = '127.0.0.1'
                input_port = 5000
            
            output_port = args.stream_output_port
            
            print("Starting stream overlay pipeline...", file=sys.stderr)
            print("  Input: tcp://{}:{}".format(input_host, input_port), file=sys.stderr)
            print("  Output: tcp://0.0.0.0:{}".format(output_port), file=sys.stderr)
            print("  Press Ctrl+C to stop", file=sys.stderr)
            
            import threading
            import queue
            
            # First, determine frame size by reading from stream directly
            print("Determining input stream dimensions...", file=sys.stderr)
            test_pipeline = "tcpclientsrc host={} port={} ! h264parse ! avdec_h264 ! videoconvert ! appsink".format(input_host, input_port)
            test_cap = cv2.VideoCapture(test_pipeline, cv2.CAP_GSTREAMER)
            
            if not test_cap.isOpened():
                print("Error: Could not open input stream for testing", file=sys.stderr)
                sys.exit(1)
            
            test_cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
            time.sleep(2.0)  # Wait for stream
            
            frame_size = None
            for attempt in range(30):
                ret, test_frame = test_cap.read()
                if ret and test_frame is not None:
                    h, w = test_frame.shape[:2]
                    frame_size = (w, h)
                    print("Input stream dimensions: {}x{}".format(w, h), file=sys.stderr)
                    break
                if attempt % 5 == 0:
                    print("Waiting for input stream... attempt {}/30".format(attempt + 1), file=sys.stderr)
                time.sleep(0.2)
            
            test_cap.release()
            
            if frame_size is None:
                print("Error: Could not determine input stream dimensions", file=sys.stderr)
                sys.exit(1)
            
            width, height = frame_size
            
            frame_queue = queue.Queue(maxsize=2)
            sharpness_value = [0.0]
            focus_value = [0]
            focus_change_queue = queue.Queue()  # Queue for focus change commands
            
            def keyboard_listener_worker():
                """Worker thread to listen for keyboard input and change focus"""
                import select
                import termios
                import tty
                
                # Setup non-blocking input
                old_settings = None
                try:
                    old_settings = termios.tcgetattr(sys.stdin)
                    tty.cbreak(sys.stdin.fileno())
                except:
                    print("Warning: Could not setup keyboard input (running as service?)", file=sys.stderr)
                    return
                
                try:
                    print("Keyboard controls active: Arrow Up/Down or +/- to adjust focus, Q to quit", file=sys.stderr)
                    while True:
                        if select.select([sys.stdin], [], [], 0.1)[0]:
                            try:
                                key = sys.stdin.read(1)
                                if key.lower() == 'q':
                                    focus_change_queue.put('quit')
                                    break
                                elif key == '+' or key == '=':
                                    focus_change_queue.put('up')
                                elif key == '-' or key == '_':
                                    focus_change_queue.put('down')
                                elif key == '\x1b':  # ESC sequence start
                                    # Check for arrow keys
                                    if select.select([sys.stdin], [], [], 0.1)[0]:
                                        seq = sys.stdin.read(2)
                                        if seq == '[A':  # Arrow Up
                                            focus_change_queue.put('up')
                                        elif seq == '[B':  # Arrow Down
                                            focus_change_queue.put('down')
                            except:
                                pass
                finally:
                    if old_settings:
                        try:
                            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, old_settings)
                        except:
                            pass
            
            def measure_sharpness_worker():
                """Worker thread to continuously measure sharpness and overlay"""
                pipeline = "tcpclientsrc host={} port={} ! h264parse ! avdec_h264 ! videoconvert ! appsink".format(input_host, input_port)
                cap = cv2.VideoCapture(pipeline, cv2.CAP_GSTREAMER)
                
                if not cap.isOpened():
                    print("Error: Could not open input stream", file=sys.stderr)
                    return
                
                cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
                time.sleep(1.0)  # Wait for stream
                
                while True:
                    ret, frame = cap.read()
                    if ret and frame is not None:
                        # Measure sharpness
                        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
                        sharpness = cv2.Laplacian(gray, cv2.CV_64F).var()
                        sharpness_value[0] = sharpness
                        
                        # Get current focus
                        try:
                            current_focus = focuser.get(Focuser.OPT_FOCUS, read_hardware=True)
                            if current_focus is None:
                                current_focus = focuser.get(Focuser.OPT_FOCUS, read_hardware=False) or 0
                            focus_value[0] = current_focus
                        except:
                            pass
                        
                        # Overlay text on frame
                        h, w = frame.shape[:2]
                        font = cv2.FONT_HERSHEY_SIMPLEX
                        font_scale = 1.0
                        thickness = 2
                        
                        # Background for text
                        overlay = frame.copy()
                        cv2.rectangle(overlay, (10, 10), (400, 100), (0, 0, 0), -1)
                        cv2.addWeighted(overlay, 0.7, frame, 0.3, 0, frame)
                        
                        # Text color based on sharpness
                        if sharpness > 100:
                            text_color = (0, 255, 0)  # Green
                        elif sharpness > 50:
                            text_color = (0, 255, 255)  # Yellow
                        else:
                            text_color = (0, 0, 255)  # Red
                        
                        cv2.putText(frame, "Focus: {}".format(focus_value[0]), 
                                   (20, 40), font, font_scale, (255, 255, 255), thickness)
                        cv2.putText(frame, "Sharpness: {:.2f}".format(sharpness), 
                                   (20, 80), font, font_scale, text_color, thickness)
                        
                        # Put frame in queue
                        try:
                            frame_queue.put_nowait(frame)
                        except queue.Full:
                            try:
                                frame_queue.get_nowait()
                                frame_queue.put_nowait(frame)
                            except:
                                pass
                    time.sleep(0.033)
            
            measure_thread = threading.Thread(target=measure_sharpness_worker, daemon=True)
            measure_thread.start()
            
            keyboard_thread = threading.Thread(target=keyboard_listener_worker, daemon=True)
            keyboard_thread.start()
            
            time.sleep(1.0)  # Give threads time to start
            
            # Output pipeline - use actual input dimensions
            print("Output stream ready at tcp://0.0.0.0:{}".format(output_port), file=sys.stderr)
            print("Output dimensions: {}x{}".format(width, height), file=sys.stderr)
            
            output_pipeline_str = (
                "appsrc is-live=true format=time do-timestamp=true "
                "caps=video/x-raw,format=BGR,width={},height={},framerate=17/1 ! "
                "videoconvert ! "
                "openh264enc bitrate=8000000 complexity=high gop-size=30 ! "
                "h264parse config-interval=-1 ! "
                "tcpserversink host=0.0.0.0 port={} sync=false"
            ).format(width, height, output_port)
            
            fourcc = cv2.VideoWriter_fourcc(*'H264')
            out = cv2.VideoWriter(output_pipeline_str, cv2.CAP_GSTREAMER, 17.0, (width, height))
            
            if not out.isOpened():
                print("Error: Could not open output pipeline", file=sys.stderr)
                print("Pipeline: {}".format(output_pipeline_str), file=sys.stderr)
                sys.exit(1)
            
            print("Stream overlay active. Connect with:")
            print("  ffmpeg -f h264 -i tcp://aspace-prod-19:{} -c:v copy -f mpegts - | ffplay -".format(output_port))
            
            try:
                while True:
                    # Check for focus change commands
                    try:
                        cmd = focus_change_queue.get_nowait()
                        if cmd == 'quit':
                            break
                        elif cmd == 'up':
                            current_focus = focuser.get(Focuser.OPT_FOCUS, read_hardware=True)
                            if current_focus is None:
                                current_focus = focuser.get(Focuser.OPT_FOCUS, read_hardware=False) or 0
                            new_focus = min(current_focus + args.monitor_step,
                                           focuser.opts[Focuser.OPT_FOCUS]["MAX_VALUE"])
                            focuser.set(Focuser.OPT_FOCUS, new_focus)
                            print("Focus increased to {}".format(new_focus), file=sys.stderr)
                        elif cmd == 'down':
                            current_focus = focuser.get(Focuser.OPT_FOCUS, read_hardware=True)
                            if current_focus is None:
                                current_focus = focuser.get(Focuser.OPT_FOCUS, read_hardware=False) or 0
                            new_focus = max(current_focus - args.monitor_step, 0)
                            focuser.set(Focuser.OPT_FOCUS, new_focus)
                            print("Focus decreased to {}".format(new_focus), file=sys.stderr)
                    except queue.Empty:
                        pass
                    
                    # Write frames
                    try:
                        frame = frame_queue.get(timeout=1.0)
                        out.write(frame)
                    except queue.Empty:
                        continue
            except KeyboardInterrupt:
                print("\nStopping overlay pipeline...", file=sys.stderr)
            
            out.release()
            print("Overlay pipeline stopped")
            sys.exit(0)
            
        except Exception as e:
            print("Error in overlay pipeline: {}".format(e), file=sys.stderr)
            import traceback
            if args.verbose:
                traceback.print_exc()
            sys.exit(1)
    
    # Handle step-up
    if args.step_up:
        key = args.step_up.upper()
        try:
            # Check if it's a camera parameter
            if key in ['EXPOSURE', 'GAIN']:
                # Try V4L2 first (works directly with IMX519 via nvargus plugin)
                current_value = None
                if v4l2_control:
                    current_value = v4l2_control.get(key)
                    if current_value is not None:
                        min_val, max_val = v4l2_control.get_range(key)
                        new_value = min(current_value + args.step, max_val if max_val is not None else current_value + args.step)
                        if v4l2_control.set(key, int(new_value)):
                            if args.verbose:
                                print("Increased {} from {} to {} (step: {})".format(key, int(current_value), int(new_value), args.step))
                            else:
                                print(int(new_value))
                            sys.exit(0)
                
                # Fall back to nvargus
                if nvargus_control:
                    current_value = nvargus_control.get(key)
                    if current_value is not None:
                        min_val, max_val = nvargus_control.get_range(key)
                        new_value = min(current_value + args.step, max_val if max_val is not None else current_value + args.step)
                        if nvargus_control.set(key, int(new_value)):
                            if args.verbose:
                                print("Increased {} from {} to {} (step: {})".format(key, int(current_value), int(new_value), args.step))
                            else:
                                print(int(new_value))
                            sys.exit(0)
                
                if current_value is None:
                    print("Error: Could not get current value for {}".format(key), file=sys.stderr)
                    sys.exit(1)
                print("Error: Failed to set {}={}".format(key, int(new_value)), file=sys.stderr)
                sys.exit(1)
            elif key in ['CONTRAST', 'BRIGHTNESS', 'SATURATION', 'HUE', 
                         'WHITE_BALANCE', 'SHARPNESS', 'GAMMA']:
                # V4L2 only parameters
                if not v4l2_control:
                    print("Error: V4L2 control not available", file=sys.stderr)
                    sys.exit(1)
                current_value = v4l2_control.get(key)
                if current_value is None:
                    print("Error: Could not get current value for {}".format(key), file=sys.stderr)
                    sys.exit(1)
                min_val, max_val = v4l2_control.get_range(key)
                new_value = min(current_value + args.step, max_val if max_val is not None else current_value + args.step)
                if v4l2_control.set(key, int(new_value)):
                    if args.verbose:
                        print("Increased {} from {} to {} (step: {})".format(key, int(current_value), int(new_value), args.step))
                    else:
                        print(int(new_value))
                    sys.exit(0)
                else:
                    print("Error: Failed to set {}={}".format(key, int(new_value)), file=sys.stderr)
                    sys.exit(1)
            else:
                # I2C parameter
                opt = get_opt_constant(key)
                if opt is None:
                    print("Error: Unknown parameter '{}'".format(key), file=sys.stderr)
                    sys.exit(1)
                # Try to read from hardware first, fall back to cached value
                current_value = focuser.get(opt, read_hardware=args.read_hardware)
                if current_value is None:
                    current_value = focuser.get(opt, read_hardware=False)
                new_value = min(current_value + args.step, focuser.opts[opt]["MAX_VALUE"])
                # Apply step validation if enabled
                if key == 'FOCUS' and args.validate_step > 0:
                    new_value = round(new_value / args.validate_step) * args.validate_step
                focuser.set(opt, new_value)
                if args.verbose:
                    print("Increased {} from {} to {} (step: {})".format(key, current_value, new_value, args.step))
                else:
                    print(new_value)
                sys.exit(0)
        except Exception as e:
            print("Error: Failed to step up {}: {}".format(key, e), file=sys.stderr)
            sys.exit(1)
    
    # Handle step-down
    if args.step_down:
        key = args.step_down.upper()
        try:
            # Check if it's a camera parameter
            if key in ['EXPOSURE', 'GAIN']:
                # Try V4L2 first (works directly with IMX519 via nvargus plugin)
                current_value = None
                if v4l2_control:
                    current_value = v4l2_control.get(key)
                    if current_value is not None:
                        min_val, max_val = v4l2_control.get_range(key)
                        new_value = max(current_value - args.step, min_val if min_val is not None else current_value - args.step)
                        if v4l2_control.set(key, int(new_value)):
                            if args.verbose:
                                print("Decreased {} from {} to {} (step: {})".format(key, int(current_value), int(new_value), args.step))
                            else:
                                print(int(new_value))
                            sys.exit(0)
                
                # Fall back to nvargus
                if nvargus_control:
                    current_value = nvargus_control.get(key)
                    if current_value is not None:
                        min_val, max_val = nvargus_control.get_range(key)
                        new_value = max(current_value - args.step, min_val if min_val is not None else current_value - args.step)
                        if nvargus_control.set(key, int(new_value)):
                            if args.verbose:
                                print("Decreased {} from {} to {} (step: {})".format(key, int(current_value), int(new_value), args.step))
                            else:
                                print(int(new_value))
                            sys.exit(0)
                
                if current_value is None:
                    print("Error: Could not get current value for {}".format(key), file=sys.stderr)
                    sys.exit(1)
                print("Error: Failed to set {}={}".format(key, int(new_value)), file=sys.stderr)
                sys.exit(1)
            elif key in ['CONTRAST', 'BRIGHTNESS', 'SATURATION', 'HUE', 
                         'WHITE_BALANCE', 'SHARPNESS', 'GAMMA']:
                # V4L2 only parameters
                if not v4l2_control:
                    print("Error: V4L2 control not available", file=sys.stderr)
                    sys.exit(1)
                current_value = v4l2_control.get(key)
                if current_value is None:
                    print("Error: Could not get current value for {}".format(key), file=sys.stderr)
                    sys.exit(1)
                min_val, max_val = v4l2_control.get_range(key)
                new_value = max(current_value - args.step, min_val if min_val is not None else current_value - args.step)
                if v4l2_control.set(key, int(new_value)):
                    if args.verbose:
                        print("Decreased {} from {} to {} (step: {})".format(key, int(current_value), int(new_value), args.step))
                    else:
                        print(int(new_value))
                    sys.exit(0)
                else:
                    print("Error: Failed to set {}={}".format(key, int(new_value)), file=sys.stderr)
                    sys.exit(1)
            else:
                # I2C parameter
                opt = get_opt_constant(key)
                if opt is None:
                    print("Error: Unknown parameter '{}'".format(key), file=sys.stderr)
                    sys.exit(1)
                # Try to read from hardware first, fall back to cached value
                current_value = focuser.get(opt, read_hardware=args.read_hardware)
                if current_value is None:
                    current_value = focuser.get(opt, read_hardware=False)
                new_value = max(current_value - args.step, focuser.opts[opt]["MIN_VALUE"])
                # Apply step validation if enabled
                if key == 'FOCUS' and args.validate_step > 0:
                    new_value = round(new_value / args.validate_step) * args.validate_step
                focuser.set(opt, new_value)
                if args.verbose:
                    print("Decreased {} from {} to {} (step: {})".format(key, current_value, new_value, args.step))
                else:
                    print(new_value)
                sys.exit(0)
        except Exception as e:
            print("Error: Failed to step down {}: {}".format(key, e), file=sys.stderr)
            sys.exit(1)
    
    # Handle text-only monitor - for use with external video player
    if args.monitor_text:
        if not CV2_AVAILABLE:
            print("Error: Text monitor requires OpenCV and numpy.", file=sys.stderr)
            print("Install with: pip install opencv-python numpy", file=sys.stderr)
            sys.exit(1)
        
        try:
            import select
            import termios
            import tty
        except ImportError:
            print("Warning: select/termios not available, keyboard input may not work", file=sys.stderr)
            select = None
            termios = None
            tty = None
        
        try:
            # Parse ROI
            try:
                roi_parts = [float(x) for x in args.monitor_roi.split(',')]
                if len(roi_parts) != 4:
                    raise ValueError("ROI must have 4 values")
                roi = tuple(roi_parts)
            except (ValueError, AttributeError):
                print("Warning: Invalid ROI format, using default center ROI", file=sys.stderr)
                roi = (0.4, 0.4, 0.2, 0.2)
            
            # Parse URL to get host and port
            if args.stream_url.startswith('tcp://'):
                url_parts = args.stream_url.replace('tcp://', '').split(':')
                host = url_parts[0] if len(url_parts) > 0 else '127.0.0.1'
                port = int(url_parts[1]) if len(url_parts) > 1 else 5000
            else:
                host = '127.0.0.1'
                port = 5000
            
            # Get current focus
            current_focus = focuser.get(Focuser.OPT_FOCUS, read_hardware=True)
            if current_focus is None:
                current_focus = focuser.get(Focuser.OPT_FOCUS, read_hardware=False) or 0
            
            print("Starting text-only focus monitor...")
            print("  Stream: {}:{}".format(host, port))
            print("  Current focus: {}".format(current_focus))
            print("  Step size: {}".format(args.monitor_step))
            print("  Update interval: {}s".format(args.monitor_interval))
            print("  ROI: x={:.1f}, y={:.1f}, w={:.1f}, h={:.1f}".format(roi[0], roi[1], roi[2], roi[3]))
            print("\nControls:")
            print("  + : Increase focus")
            print("  - : Decrease focus")
            print("  Q : Quit")
            print("\nWatching stream and measuring sharpness...\n")
            
            # GStreamer pipeline for TCP H.264 stream
            pipeline = "tcpclientsrc host={} port={} ! h264parse ! avdec_h264 ! videoconvert ! appsink".format(host, port)
            cap = cv2.VideoCapture(pipeline, cv2.CAP_GSTREAMER)
            
            if not cap.isOpened():
                print("Error: Could not open camera stream at {}:{}".format(host, port), file=sys.stderr)
                print("Make sure camera stream is running: ss -tlnp | grep :{}".format(port), file=sys.stderr)
                sys.exit(1)
            
            cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
            
            # Wait for stream
            print("Waiting for stream...")
            for i in range(20):
                ret, test_frame = cap.read()
                if ret and test_frame is not None and test_frame.size > 0:
                    print("Stream ready!\n")
                    break
                time.sleep(0.1)
            else:
                print("Warning: Could not read initial frames", file=sys.stderr)
            
            # Laplacian sharpness function
            def laplacian_sharpness(img):
                gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
                return cv2.Laplacian(gray, cv2.CV_64F).var()
            
            def get_roi_frame(frame, roi):
                h, w = frame.shape[:2]
                x_start = int(w * roi[0])
                x_end = x_start + int(w * roi[2])
                y_start = int(h * roi[1])
                y_end = y_start + int(h * roi[3])
                return frame[y_start:y_end, x_start:x_end]
            
            last_focus = current_focus
            last_sharpness = None
            sharpness_history = []
            max_history = 10
            
            # Setup non-blocking input if possible
            old_settings = None
            if termios and tty:
                try:
                    old_settings = termios.tcgetattr(sys.stdin)
                    tty.cbreak(sys.stdin.fileno())
                except:
                    pass
            
            try:
                frame_count = 0
                last_update = time.time()
                
                while True:
                    # Check for keyboard input (non-blocking)
                    key_pressed = None
                    if select and sys.stdin in select.select([sys.stdin], [], [], 0)[0]:
                        try:
                            key_pressed = sys.stdin.read(1)
                        except:
                            pass
                    
                    if key_pressed:
                        if key_pressed.lower() == 'q':
                            break
                        elif key_pressed == '+' or key_pressed == '=':
                            new_focus = min(current_focus + args.monitor_step,
                                           focuser.opts[Focuser.OPT_FOCUS]["MAX_VALUE"])
                            focuser.set(Focuser.OPT_FOCUS, new_focus)
                            time.sleep(0.3)
                            current_focus = new_focus
                            sharpness_history.clear()
                            print("Focus increased to {}".format(new_focus))
                        elif key_pressed == '-' or key_pressed == '_':
                            new_focus = max(current_focus - args.monitor_step, 0)
                            focuser.set(Focuser.OPT_FOCUS, new_focus)
                            time.sleep(0.3)
                            current_focus = new_focus
                            sharpness_history.clear()
                            print("Focus decreased to {}".format(new_focus))
                    
                    # Read frame and measure sharpness
                    ret, frame = cap.read()
                    if not ret or frame is None:
                        time.sleep(0.1)
                        continue
                    
                    frame_count += 1
                    current_time = time.time()
                    
                    # Update at specified interval
                    if current_time - last_update >= args.monitor_interval:
                        roi_frame = get_roi_frame(frame, roi)
                        if roi_frame.size > 0:
                            sharpness = laplacian_sharpness(roi_frame)
                            last_sharpness = sharpness
                            sharpness_history.append(sharpness)
                            if len(sharpness_history) > max_history:
                                sharpness_history.pop(0)
                            avg_sharpness = sum(sharpness_history) / len(sharpness_history)
                            
                            # Get current focus
                            current_focus = focuser.get(Focuser.OPT_FOCUS, read_hardware=True)
                            if current_focus is None:
                                current_focus = focuser.get(Focuser.OPT_FOCUS, read_hardware=False) or 0
                            
                            # Calculate change indicators
                            focus_changed = (current_focus != last_focus)
                            focus_delta = current_focus - last_focus if focus_changed else 0
                            
                            if len(sharpness_history) > 1:
                                min_sharp = min(sharpness_history)
                                max_sharp = max(sharpness_history)
                                range_info = "Range: {:.2f}-{:.2f}".format(min_sharp, max_sharp)
                            else:
                                range_info = ""
                            
                            # Print status line (overwrite previous)
                            status = "\rFocus: {:4d} | Sharpness: {:7.2f} | {}".format(
                                current_focus, avg_sharpness, range_info)
                            if focus_changed:
                                status += " | Change: {:+d}".format(focus_delta)
                            
                            # Color coding with ANSI codes
                            if avg_sharpness > 100:
                                color_code = "\033[92m"  # Green
                            elif avg_sharpness > 50:
                                color_code = "\033[93m"  # Yellow
                            else:
                                color_code = "\033[91m"  # Red
                            
                            reset_code = "\033[0m"
                            print("{}{}{}".format(color_code, status, reset_code), end='', flush=True)
                            
                            last_focus = current_focus
                            last_update = current_time
                    
                    time.sleep(0.05)  # Small sleep to prevent CPU spinning
                
            finally:
                # Restore terminal settings
                if old_settings:
                    try:
                        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, old_settings)
                    except:
                        pass
                print("\n")  # New line after status
            
            cap.release()
            
            # Get final focus value
            final_focus = focuser.get(Focuser.OPT_FOCUS, read_hardware=True)
            if final_focus is None:
                final_focus = focuser.get(Focuser.OPT_FOCUS, read_hardware=False) or 0
            
            print("Monitor stopped. Final focus: {}".format(final_focus))
            print(final_focus)
            sys.exit(0)
            
        except KeyboardInterrupt:
            print("\nMonitor interrupted by user", file=sys.stderr)
            if 'cap' in locals():
                cap.release()
            sys.exit(0)
        except Exception as e:
            print("Error during text monitor: {}".format(e), file=sys.stderr)
            import traceback
            if args.verbose:
                traceback.print_exc()
            if 'cap' in locals():
                cap.release()
            sys.exit(1)
    
    # Handle live focus monitor - real-time sharpness display
    if args.monitor:
        if not CV2_AVAILABLE:
            print("Error: Monitor mode requires OpenCV and numpy.", file=sys.stderr)
            print("Install with: pip install opencv-python numpy", file=sys.stderr)
            sys.exit(1)
        
        try:
            # Parse ROI
            try:
                roi_parts = [float(x) for x in args.monitor_roi.split(',')]
                if len(roi_parts) != 4:
                    raise ValueError("ROI must have 4 values")
                roi = tuple(roi_parts)
            except (ValueError, AttributeError):
                print("Warning: Invalid ROI format, using default center ROI", file=sys.stderr)
                roi = (0.4, 0.4, 0.2, 0.2)
            
            # Parse URL to get host and port
            if args.stream_url.startswith('tcp://'):
                url_parts = args.stream_url.replace('tcp://', '').split(':')
                host = url_parts[0] if len(url_parts) > 0 else '127.0.0.1'
                port = int(url_parts[1]) if len(url_parts) > 1 else 5000
            else:
                host = '127.0.0.1'
                port = 5000
            
            # Get current focus
            current_focus = focuser.get(Focuser.OPT_FOCUS, read_hardware=True)
            if current_focus is None:
                current_focus = focuser.get(Focuser.OPT_FOCUS, read_hardware=False) or 0
            
            print("Starting live focus monitor...")
            print("  Stream: {}:{}".format(host, port))
            print("  Current focus: {}".format(current_focus))
            print("  Step size: {}".format(args.monitor_step))
            print("  ROI: x={:.1f}, y={:.1f}, w={:.1f}, h={:.1f}".format(roi[0], roi[1], roi[2], roi[3]))
            print("\nControls:")
            print("  Arrow Up / + : Increase focus")
            print("  Arrow Down / - : Decrease focus")
            print("  Space: Refresh sharpness measurement")
            print("  Q / Esc: Quit")
            print("\nPress any key in the window to start...")
            
            # GStreamer pipeline for TCP H.264 stream
            pipeline = "tcpclientsrc host={} port={} ! h264parse ! avdec_h264 ! videoconvert ! appsink".format(host, port)
            if args.verbose:
                print("GStreamer pipeline: {}".format(pipeline))
            
            cap = cv2.VideoCapture(pipeline, cv2.CAP_GSTREAMER)
            
            if not cap.isOpened():
                print("Error: Could not open camera stream at {}:{}".format(host, port), file=sys.stderr)
                print("Make sure camera stream is running: ss -tlnp | grep :{}".format(port), file=sys.stderr)
                sys.exit(1)
            
            # Set buffer size to reduce latency
            cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
            
            # Wait for stream to stabilize and verify we can read frames
            print("Waiting for stream to stabilize...")
            frames_read = 0
            for i in range(20):
                ret, test_frame = cap.read()
                if ret and test_frame is not None and test_frame.size > 0:
                    frames_read += 1
                    if frames_read >= 3:  # Got a few good frames
                        print("Stream ready! Frame shape: {}".format(test_frame.shape))
                        break
                time.sleep(0.1)
            else:
                print("Warning: Could not read initial frames, continuing anyway...", file=sys.stderr)
            
            # Laplacian sharpness function
            def laplacian_sharpness(img):
                """Calculate image sharpness using Laplacian variance - higher = sharper"""
                gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
                laplacian_var = cv2.Laplacian(gray, cv2.CV_64F).var()
                return laplacian_var
            
            # Get ROI from frame
            def get_roi_frame(frame, roi):
                h, w = frame.shape[:2]
                x_start = int(w * roi[0])
                x_end = x_start + int(w * roi[2])
                y_start = int(h * roi[1])
                y_end = y_start + int(h * roi[3])
                return frame[y_start:y_end, x_start:x_end]
            
            # Draw ROI rectangle on frame
            def draw_roi(frame, roi, color=(0, 255, 0), thickness=2):
                h, w = frame.shape[:2]
                x_start = int(w * roi[0])
                x_end = x_start + int(w * roi[2])
                y_start = int(h * roi[1])
                y_end = y_start + int(h * roi[3])
                cv2.rectangle(frame, (x_start, y_start), (x_end, y_end), color, thickness)
                return frame
            
            window_name = "Focus Monitor - Press Q to quit"
            try:
                cv2.namedWindow(window_name, cv2.WINDOW_NORMAL)
                cv2.resizeWindow(window_name, 1280, 720)  # Set initial size
                print("Display window created successfully")
            except Exception as e:
                print("Warning: Could not create display window: {}".format(e), file=sys.stderr)
                print("If running via SSH, make sure X11 forwarding is enabled:", file=sys.stderr)
                print("  ssh -X user@host", file=sys.stderr)
                print("Or set DISPLAY: export DISPLAY=:0", file=sys.stderr)
            
            last_sharpness = None
            sharpness_history = []
            max_history = 10
            
            # Track changes for visual feedback
            last_focus = current_focus
            focus_change_time = 0
            last_sharpness_value = None
            sharpness_change_time = 0
            change_display_duration = 1.0  # Show change indicator for 1 second
            
            frame_count = 0
            while True:
                ret, frame = cap.read()
                if not ret:
                    print("Warning: Could not read frame", file=sys.stderr)
                    time.sleep(0.1)
                    continue
                
                # Debug: Check frame validity
                if frame is None or frame.size == 0:
                    print("Warning: Empty frame received", file=sys.stderr)
                    time.sleep(0.1)
                    continue
                
                frame_count += 1
                if frame_count == 1:
                    print("First frame received: shape={}, dtype={}".format(frame.shape, frame.dtype))
                
                # Get ROI and measure sharpness
                roi_frame = get_roi_frame(frame, roi)
                if roi_frame.size > 0:
                    sharpness = laplacian_sharpness(roi_frame)
                    last_sharpness = sharpness
                    sharpness_history.append(sharpness)
                    if len(sharpness_history) > max_history:
                        sharpness_history.pop(0)
                    avg_sharpness = sum(sharpness_history) / len(sharpness_history)
                else:
                    avg_sharpness = last_sharpness or 0
                
                # Draw ROI rectangle
                frame = draw_roi(frame, roi)
                
                # Get current focus value
                current_focus = focuser.get(Focuser.OPT_FOCUS, read_hardware=True)
                if current_focus is None:
                    current_focus = focuser.get(Focuser.OPT_FOCUS, read_hardware=False) or 0
                
                # Track changes
                current_time = time.time()
                focus_changed = (current_focus != last_focus)
                focus_delta = 0
                if focus_changed:
                    focus_delta = current_focus - last_focus
                    focus_change_time = current_time
                    last_focus = current_focus
                
                sharpness_changed = False
                sharpness_delta = 0
                if last_sharpness_value is not None:
                    sharpness_delta = avg_sharpness - last_sharpness_value
                    sharpness_changed = abs(sharpness_delta) > 2.0
                    if sharpness_changed:
                        sharpness_change_time = current_time
                last_sharpness_value = avg_sharpness
                
                # Overlay text information
                h, w = frame.shape[:2]
                font = cv2.FONT_HERSHEY_SIMPLEX
                font_scale = 1.0
                thickness = 2
                
                # Background for text (semi-transparent) - make it wider for more info
                overlay = frame.copy()
                cv2.rectangle(overlay, (10, 10), (600, 220), (0, 0, 0), -1)
                cv2.addWeighted(overlay, 0.7, frame, 0.3, 0, frame)
                
                # Text color based on sharpness (green = good, red = bad)
                if avg_sharpness > 100:
                    text_color = (0, 255, 0)  # Green
                elif avg_sharpness > 50:
                    text_color = (0, 255, 255)  # Yellow
                else:
                    text_color = (0, 0, 255)  # Red
                
                # Show focus value with change indicator
                focus_text = "Focus: {}".format(current_focus)
                focus_text_color = (255, 255, 255)
                if focus_changed or (current_time - focus_change_time) < change_display_duration:
                    focus_text_color = (0, 255, 255)  # Yellow when changing
                    if focus_delta != 0:
                        focus_text += " ({:+d})".format(focus_delta)
                
                cv2.putText(frame, focus_text, 
                           (20, 40), font, font_scale, focus_text_color, thickness)
                
                # Show sharpness with change indicator
                sharpness_text = "Sharpness: {:.2f}".format(avg_sharpness)
                if sharpness_changed or (current_time - sharpness_change_time) < change_display_duration:
                    if abs(sharpness_delta) > 0.1:
                        sharpness_text += " ({:+.2f})".format(sharpness_delta)
                
                cv2.putText(frame, sharpness_text, 
                           (20, 80), font, font_scale, text_color, thickness)
                
                # Show min/max sharpness from history
                if len(sharpness_history) > 1:
                    min_sharp = min(sharpness_history)
                    max_sharp = max(sharpness_history)
                    range_text = "Range: {:.2f} - {:.2f}".format(min_sharp, max_sharp)
                    cv2.putText(frame, range_text, 
                               (20, 120), font, 0.7, (200, 200, 200), 1)
                
                # Show trend indicator (improving/worsening)
                if len(sharpness_history) >= 3:
                    recent_avg = sum(sharpness_history[-3:]) / 3
                    older_avg = sum(sharpness_history[:3]) / 3 if len(sharpness_history) >= 6 else recent_avg
                    trend = recent_avg - older_avg
                    if abs(trend) > 1.0:
                        trend_text = "Trend: {} {:.2f}".format("↑" if trend > 0 else "↓", abs(trend))
                        trend_color = (0, 255, 0) if trend > 0 else (0, 0, 255)
                        cv2.putText(frame, trend_text, 
                                   (20, 150), font, 0.7, trend_color, 1)
                
                cv2.putText(frame, "Controls: +/- or Arrow Keys to adjust, Q to quit", 
                           (20, 190), font, 0.6, (200, 200, 200), 1)
                
                # Show frame - waitKey must be called for window to update
                try:
                    cv2.imshow(window_name, frame)
                except Exception as e:
                    print("Error displaying frame: {}".format(e), file=sys.stderr)
                    print("Make sure DISPLAY is set: export DISPLAY=:0", file=sys.stderr)
                    time.sleep(0.1)
                    continue
                
                # Handle keyboard input - waitKey updates the window
                key_code = cv2.waitKey(1) & 0xFFFF
                if key_code == 0xFFFF:  # No key pressed
                    continue
                
                key = key_code & 0xFF
                
                # Check for arrow keys (they return 0 followed by extended code)
                if key_code == 82 or key_code == 65362:  # Arrow Up (different systems)
                    new_focus = min(current_focus + args.monitor_step, 
                                   focuser.opts[Focuser.OPT_FOCUS]["MAX_VALUE"])
                    focuser.set(Focuser.OPT_FOCUS, new_focus)
                    time.sleep(0.3)  # Wait for motor
                    sharpness_history.clear()  # Reset history after change
                    print("Focus increased to {}".format(new_focus))
                elif key_code == 84 or key_code == 65364:  # Arrow Down (different systems)
                    new_focus = max(current_focus - args.monitor_step, 0)
                    focuser.set(Focuser.OPT_FOCUS, new_focus)
                    time.sleep(0.3)  # Wait for motor
                    sharpness_history.clear()  # Reset history after change
                    print("Focus decreased to {}".format(new_focus))
                elif key == ord('q') or key == 27:  # Q or Esc
                    break
                elif key == ord('+') or key == ord('='):  # +
                    new_focus = min(current_focus + args.monitor_step, 
                                   focuser.opts[Focuser.OPT_FOCUS]["MAX_VALUE"])
                    focuser.set(Focuser.OPT_FOCUS, new_focus)
                    time.sleep(0.3)  # Wait for motor
                    sharpness_history.clear()  # Reset history after change
                    print("Focus increased to {}".format(new_focus))
                elif key == ord('-') or key == ord('_'):  # -
                    new_focus = max(current_focus - args.monitor_step, 0)
                    focuser.set(Focuser.OPT_FOCUS, new_focus)
                    time.sleep(0.3)  # Wait for motor
                    sharpness_history.clear()  # Reset history after change
                    print("Focus decreased to {}".format(new_focus))
                elif key == ord(' '):  # Space - refresh
                    sharpness_history.clear()
                    print("Sharpness measurement refreshed")
            
            cap.release()
            cv2.destroyAllWindows()
            
            # Get final focus value
            final_focus = focuser.get(Focuser.OPT_FOCUS, read_hardware=True)
            if final_focus is None:
                final_focus = focuser.get(Focuser.OPT_FOCUS, read_hardware=False) or 0
            
            print("\nMonitor closed. Final focus: {}".format(final_focus))
            print(final_focus)
            sys.exit(0)
            
        except KeyboardInterrupt:
            print("\nMonitor interrupted by user", file=sys.stderr)
            if 'cap' in locals():
                cap.release()
            if 'window_name' in locals():
                cv2.destroyAllWindows()
            sys.exit(0)
        except Exception as e:
            print("Error during monitor: {}".format(e), file=sys.stderr)
            import traceback
            if args.verbose:
                traceback.print_exc()
            if 'cap' in locals():
                cap.release()
            if 'window_name' in locals():
                cv2.destroyAllWindows()
            sys.exit(1)
    
    # Handle autofocus - automatic sharpness detection using OpenCV
    if args.autofocus:
        if not CV2_AVAILABLE:
            print("Error: Autofocus requires OpenCV and numpy.", file=sys.stderr)
            print("Install with: pip install opencv-python numpy", file=sys.stderr)
            sys.exit(1)
        
        try:
            # Parse ROI
            try:
                roi_parts = [float(x) for x in args.autofocus_roi.split(',')]
                if len(roi_parts) != 4:
                    raise ValueError("ROI must have 4 values")
                roi = tuple(roi_parts)
            except (ValueError, AttributeError):
                print("Warning: Invalid ROI format, using default center ROI", file=sys.stderr)
                roi = (0.4, 0.4, 0.2, 0.2)  # Default center 20%
            
            # Parse URL to get host and port
            if args.stream_url.startswith('tcp://'):
                url_parts = args.stream_url.replace('tcp://', '').split(':')
                host = url_parts[0] if len(url_parts) > 0 else '127.0.0.1'
                port = int(url_parts[1]) if len(url_parts) > 1 else 5000
            else:
                host = '127.0.0.1'
                port = 5000
            
            if args.verbose:
                print("Starting autofocus with automatic sharpness detection...")
                print("  Stream: {}:{}".format(host, port))
                print("  Range: 0 to {} (step: {})".format(focuser.opts[Focuser.OPT_FOCUS]["MAX_VALUE"], args.autofocus_step))
                print("  Samples per position: {}".format(args.autofocus_samples))
                print("  ROI: x={:.1f}, y={:.1f}, w={:.1f}, h={:.1f}".format(roi[0], roi[1], roi[2], roi[3]))
            
            # GStreamer pipeline for TCP H.264 stream
            pipeline = "tcpclientsrc host={} port={} ! h264parse ! avdec_h264 ! videoconvert ! appsink".format(host, port)
            cap = cv2.VideoCapture(pipeline, cv2.CAP_GSTREAMER)
            
            if not cap.isOpened():
                print("Error: Could not open camera stream at {}:{}".format(host, port), file=sys.stderr)
                print("Make sure camera stream is running: ss -tlnp | grep :{}".format(port), file=sys.stderr)
                sys.exit(1)
            
            # Wait a bit for stream to stabilize
            time.sleep(1.0)
            
            # Laplacian sharpness function - best method for focus detection
            def laplacian_sharpness(img):
                """Calculate image sharpness using Laplacian variance - higher = sharper"""
                gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
                laplacian_var = cv2.Laplacian(gray, cv2.CV_64F).var()
                return laplacian_var
            
            # Get ROI from frame
            def get_roi_frame(frame, roi):
                h, w = frame.shape[:2]
                x_start = int(w * roi[0])
                x_end = x_start + int(w * roi[2])
                y_start = int(h * roi[1])
                y_end = y_start + int(h * roi[3])
                return frame[y_start:y_end, x_start:x_end]
            
            # Measure sharpness at a focus position
            def measure_sharpness(focus_pos, cap, roi, samples, verbose=False):
                """Set focus and measure sharpness"""
                focuser.set(Focuser.OPT_FOCUS, focus_pos)
                # Wait for motor to move and image to stabilize
                time.sleep(1.2)  # Slightly longer for better stabilization
                
                sharpness_values = []
                failed_reads = 0
                
                for sample in range(samples):
                    ret, frame = cap.read()
                    if not ret:
                        failed_reads += 1
                        if failed_reads >= samples:
                            if verbose:
                                print("Warning: Could not read frames at position {}".format(focus_pos))
                            break
                        time.sleep(0.2)
                        continue
                    
                    roi_frame = get_roi_frame(frame, roi)
                    if roi_frame.size > 0:
                        sharpness = laplacian_sharpness(roi_frame)
                        sharpness_values.append(sharpness)
                
                if sharpness_values:
                    # Use median for robustness against outliers
                    sorted_sharpness = sorted(sharpness_values)
                    median_sharpness = sorted_sharpness[len(sorted_sharpness) // 2]
                    return median_sharpness
                return None
            
            min_focus = 0
            max_focus = focuser.opts[Focuser.OPT_FOCUS]["MAX_VALUE"]
            coarse_step = args.autofocus_step
            fine_step = args.autofocus_fine_step
            refinement_range = args.autofocus_refinement_range
            
            sharpness_results = []
            start_time = time.time()
            
            if args.verbose:
                print("\n" + "="*50)
                print("PHASE 1: Coarse scan (step: {})".format(coarse_step))
                print("="*50)
            
            # PHASE 1: Coarse scan through entire range
            for focus_pos in range(min_focus, max_focus + coarse_step, coarse_step):
                if focus_pos > max_focus:
                    focus_pos = max_focus
                
                # Check timeout
                if time.time() - start_time > args.autofocus_timeout:
                    if args.verbose:
                        print("Warning: Autofocus timeout reached", file=sys.stderr)
                    break
                
                sharpness = measure_sharpness(focus_pos, cap, roi, args.autofocus_samples, args.verbose)
                
                if sharpness is not None:
                    sharpness_results.append((focus_pos, sharpness))
                    if args.verbose:
                        print("  Focus {:4d}: sharpness = {:8.2f}".format(focus_pos, sharpness))
            
            if not sharpness_results:
                cap.release()
                print("Error: No sharpness measurements obtained. Check camera stream.", file=sys.stderr)
                sys.exit(1)
            
            # Find best position from coarse scan
            coarse_best_pos, coarse_best_sharpness = max(sharpness_results, key=lambda x: x[1])
            
            if args.verbose:
                print("\nCoarse scan complete. Best position: {} (sharpness: {:.2f})".format(
                    coarse_best_pos, coarse_best_sharpness))
            
            # PHASE 2: Fine scan around best position
            refinement_half_range = refinement_range // 2
            fine_min = max(min_focus, coarse_best_pos - refinement_half_range)
            fine_max = min(max_focus, coarse_best_pos + refinement_half_range)
            
            if args.verbose:
                print("\n" + "="*50)
                print("PHASE 2: Fine refinement (step: {}, range: {} to {})".format(
                    fine_step, fine_min, fine_max))
                print("="*50)
            
            fine_results = []
            
            # Test fine positions around best
            for focus_pos in range(fine_min, fine_max + fine_step, fine_step):
                if focus_pos > fine_max:
                    focus_pos = fine_max
                
                # Skip if we already tested this position in coarse scan
                if any(pos == focus_pos for pos, _ in sharpness_results):
                    continue
                
                # Check timeout
                if time.time() - start_time > args.autofocus_timeout:
                    if args.verbose:
                        print("Warning: Autofocus timeout reached during refinement", file=sys.stderr)
                    break
                
                sharpness = measure_sharpness(focus_pos, cap, roi, args.autofocus_samples, args.verbose)
                
                if sharpness is not None:
                    fine_results.append((focus_pos, sharpness))
                    if args.verbose:
                        print("  Focus {:4d}: sharpness = {:8.2f}".format(focus_pos, sharpness))
            
            # Combine results
            all_results = sharpness_results + fine_results
            
            if not all_results:
                cap.release()
                print("Error: No sharpness measurements obtained.", file=sys.stderr)
                sys.exit(1)
            
            # Find overall best position
            best_pos, best_sharpness = max(all_results, key=lambda x: x[1])
            
            # Also find second best for comparison
            sorted_results = sorted(all_results, key=lambda x: x[1], reverse=True)
            second_best = sorted_results[1] if len(sorted_results) > 1 else None
            
            cap.release()
            
            if args.verbose:
                print("\n" + "="*50)
                print("Autofocus Results:")
                print("  Coarse best: {} (sharpness: {:.2f})".format(coarse_best_pos, coarse_best_sharpness))
                print("  Final best: {} (sharpness: {:.2f})".format(best_pos, best_sharpness))
                if second_best:
                    print("  2nd best: {} (sharpness: {:.2f})".format(second_best[0], second_best[1]))
                print("  Total positions tested: {} (coarse: {}, fine: {})".format(
                    len(all_results), len(sharpness_results), len(fine_results)))
                print("="*50)
                print("Setting focus to {}...".format(best_pos))
            
            # Set to best focus
            focuser.set(Focuser.OPT_FOCUS, best_pos)
            time.sleep(0.5)
            
            # Verify final position
            final_value = focuser.get(Focuser.OPT_FOCUS, read_hardware=True)
            if final_value is None:
                final_value = focuser.get(Focuser.OPT_FOCUS, read_hardware=False)
            
            if args.verbose:
                print("Focus set to {} (verified: {})".format(best_pos, final_value))
            
            print(best_pos)
            sys.exit(0)
            
        except KeyboardInterrupt:
            print("\nAutofocus interrupted by user", file=sys.stderr)
            current = focuser.get(Focuser.OPT_FOCUS, read_hardware=True)
            if current is None:
                current = focuser.get(Focuser.OPT_FOCUS, read_hardware=False)
            if current is not None:
                print("Current focus position: {}".format(current))
            if 'cap' in locals():
                cap.release()
            sys.exit(0)
        except Exception as e:
            print("Error during autofocus: {}".format(e), file=sys.stderr)
            import traceback
            if args.verbose:
                traceback.print_exc()
            if 'cap' in locals():
                cap.release()
            sys.exit(1)
    
    # Handle reset
    if args.reset:
        try:
            focuser.reset(Focuser.OPT_FOCUS)
            if args.verbose:
                print("Focus reset to default value")
            sys.exit(0)
        except Exception as e:
            print("Error: Failed to reset focus: {}".format(e), file=sys.stderr)
            sys.exit(1)
    
    # Parse and set values
    if not args.values:
        print("Error: No values specified. Use --help for usage information.", file=sys.stderr)
        sys.exit(1)
    
    success = True
    for value_arg in args.values:
        if '=' not in value_arg:
            print("Error: Invalid format '{}'. Expected KEY=VALUE".format(value_arg), file=sys.stderr)
            success = False
            continue
        
        key, val = value_arg.split('=', 1)
        key = key.upper()
        
        try:
            val = int(val)
        except ValueError:
            print("Error: Invalid value '{}' for {}. Must be an integer.".format(val, key), file=sys.stderr)
            success = False
            continue
        
        # Validate step for FOCUS (if enabled)
        if key == 'FOCUS' and args.validate_step > 0:
            if val % args.validate_step != 0:
                print("Warning: FOCUS value {} is not a multiple of {}. Rounding to nearest valid value.".format(val, args.validate_step), file=sys.stderr)
                val = round(val / args.validate_step) * args.validate_step
                if args.verbose:
                    print("Using value: {}".format(val))
        
        try:
            # Handle camera parameters
            if key in ['EXPOSURE', 'GAIN']:
                # Try V4L2 first (works directly with IMX519 via nvargus plugin)
                success = False
                if v4l2_control:
                    if v4l2_control.set(key, val):
                        if args.verbose:
                            print("Set {} to {} (via V4L2)".format(key, val))
                        success = True
                
                # Fall back to nvargus if V4L2 didn't work
                if not success and nvargus_control:
                    if nvargus_control.set(key, val):
                        if args.verbose:
                            print("Set {} to {} (via nvargus)".format(key, val))
                        success = True
                
                if not success:
                    print("Warning: Failed to set {}={}. Control may not be available on this device.".format(key, val), file=sys.stderr)
            elif key in ['CONTRAST', 'BRIGHTNESS', 'SATURATION', 'HUE', 
                         'WHITE_BALANCE', 'SHARPNESS', 'GAMMA']:
                # V4L2 only parameters
                if v4l2_control:
                    if v4l2_control.set(key, val):
                        if args.verbose:
                            print("Set {} to {}".format(key, val))
                    else:
                        print("Warning: Failed to set {}={}. Control may not be available on this device.".format(key, val), file=sys.stderr)
                else:
                    print("Warning: V4L2 control not available, skipping {}".format(key), file=sys.stderr)
            # Handle I2C parameters
            elif key == 'FOCUS':
                focuser.set(Focuser.OPT_FOCUS, val)
                if args.verbose:
                    print("Set FOCUS to {}".format(val))
            elif key == 'ZOOM':
                focuser.set(Focuser.OPT_ZOOM, val)
                if args.verbose:
                    print("Set ZOOM to {}".format(val))
            elif key == 'MOTOR_X':
                focuser.set(Focuser.OPT_MOTOR_X, val)
                if args.verbose:
                    print("Set MOTOR_X to {}".format(val))
            elif key == 'MOTOR_Y':
                focuser.set(Focuser.OPT_MOTOR_Y, val)
                if args.verbose:
                    print("Set MOTOR_Y to {}".format(val))
            else:
                print("Error: Unknown parameter '{}'".format(key), file=sys.stderr)
                success = False
        except IOError as e:
            print("Warning: Failed to set {}={}: {}. Focus controller may not be responding on i2c bus.".format(key, val, e), file=sys.stderr)
            print("This is non-fatal - camera stream will continue to work.", file=sys.stderr)
            # Don't mark as failure - camera still works without focus control
            if args.verbose:
                print("Continuing despite focus control failure...", file=sys.stderr)
        except Exception as e:
            print("Error: Failed to set {}={}: {}".format(key, val, e), file=sys.stderr)
            success = False
    
    # Exit with 0 even if focus control failed - camera stream still works
    # Only exit with error if there were parsing/argument errors
    if not args.values and not args.reset:
        sys.exit(1)
    sys.exit(0)

if __name__ == "__main__":
    main()
