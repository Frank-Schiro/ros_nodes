#!/bin/bash

# Function to check if a command exists
command_exists() {
    command -v "$1" >/dev/null 2>&1
}

# Function to log messages
log() {
    echo "[$(date '+%Y-%m-%d %H:%M:%S')] $1"
}

debug() {
    if [ "$DEBUG" = "true" ]; then
        log "DEBUG: $1"
    fi
}

# Set DEBUG=true to enable debug output
DEBUG=${DEBUG:-false}

# Check for required commands
if ! command_exists lsusb; then
    log "Error: lsusb is not installed. Please install usbutils package."
    exit 1
fi

if ! command_exists usb-reset; then
    log "Error: usb-reset is not installed. Installing now..."
    sudo apt-get update && sudo apt-get install -y usb-reset
fi

# Find RealSense camera
log "Looking for RealSense camera..."
REALSENSE_INFO=$(lsusb | grep "RealSense")

if [ -z "$REALSENSE_INFO" ]; then
    log "Error: No RealSense camera found!"
    exit 1
fi

# Extract bus and device numbers
BUS=$(echo $REALSENSE_INFO | grep -o 'Bus [0-9]*' | awk '{print $2}')
DEVICE=$(echo $REALSENSE_INFO | grep -o 'Device [0-9]*' | awk '{print $2}')
VENDOR_PRODUCT=$(echo $REALSENSE_INFO | grep -o '[0-9a-f]\{4\}:[0-9a-f]\{4\}')

log "Found RealSense camera at Bus $BUS Device $DEVICE (ID $VENDOR_PRODUCT)"

# Stop Docker containers using the camera
# if command_exists docker; then
#     log "Stopping Docker containers..."
#     docker compose down 2>/dev/null || true
# fi

# If docker compose down takes more than a minute, restart Docker service
if command -v docker >/dev/null; then
    log "Stopping Docker containers..."
    
    timeout 60s docker compose down 2>/dev/null
    if [ $? -eq 124 ]; then
        log "Docker compose down took too long, resetting Docker..."
        sudo systemctl reset-failed docker
        sudo systemctl restart docker
    fi
fi


# Reset the camera
log "Resetting RealSense camera..."

# Try multiple reset methods
reset_successful=false

# Method 1: Direct device path with error output
debug "Attempting reset using device path: /dev/bus/usb/$BUS/$DEVICE"
if sudo usb-reset "/dev/bus/usb/$BUS/$DEVICE" 2>&1; then
    log "Camera reset successful using device path"
    reset_successful=true
else
    debug "Device path reset failed. Trying next method..."
fi

# Method 2: Vendor:Product ID with error output
if [ "$reset_successful" = false ]; then
    debug "Attempting reset using vendor:product ID: $VENDOR_PRODUCT"
    if sudo usb-reset "$VENDOR_PRODUCT" 2>&1; then
        log "Camera reset successful using vendor:product ID"
        reset_successful=true
    else
        debug "Vendor:Product ID reset failed. Trying next method..."
    fi
fi

# Method 3: Using unbind/bind in sysfs
if [ "$reset_successful" = false ]; then
    debug "Attempting reset using sysfs unbind/bind"
    DEVICE_PATH="/sys/bus/usb/devices/${BUS}-${DEVICE}"
    if [ -d "$DEVICE_PATH" ]; then
        if sudo sh -c "echo '$BUS-$DEVICE' > /sys/bus/usb/drivers/usb/unbind" 2>/dev/null && \
           sleep 2 && \
           sudo sh -c "echo '$BUS-$DEVICE' > /sys/bus/usb/drivers/usb/bind" 2>/dev/null; then
            log "Camera reset successful using sysfs unbind/bind"
            reset_successful=true
        else
            debug "Sysfs unbind/bind failed"
        fi
    else
        debug "Sysfs device path not found: $DEVICE_PATH"
    fi
fi

# Check if any reset method succeeded
if [ "$reset_successful" = false ]; then
    log "Error: Failed to reset camera using all available methods"
    log "Try running with DEBUG=true for more information"
    log "You may need to run: sudo chmod a+w /dev/bus/usb/$BUS/$DEVICE"
    exit 1
fi

# Wait for the camera to reconnect
log "Waiting for camera to reconnect..."
sleep 2

# Verify camera is back
if lsusb | grep -q "RealSense"; then
    log "Camera reconnected successfully"
else
    log "Warning: Cannot verify if camera reconnected. You may need to wait a few more seconds."
fi


# Setup X11 permissions if needed
if command_exists xhost; then
    if [ -n "$DISPLAY" ]; then
        log "Setting up X11 permissions..."
        xhost +local:root >/dev/null 2>&1 && \
            log "X11 permissions updated successfully" || \
            log "Warning: Failed to set X11 permissions"
    else
        debug "No X display found, skipping X11 permission setup"
    fi
else
    debug "xhost command not found, skipping X11 permission setup"
fi