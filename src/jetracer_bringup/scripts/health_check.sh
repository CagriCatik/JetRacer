#!/usr/bin/env bash
# =============================================================================
# JetRacer Bring-Up Health Check
# =============================================================================
# Validates the environment before launching the autonomous stack.
# Run from the workspace root after sourcing install/setup.bash.
#
# Usage:
#   chmod +x src/jetracer_bringup/scripts/health_check.sh
#   source install/setup.bash
#   ./src/jetracer_bringup/scripts/health_check.sh
#
# Exit code: 0 if all checks pass, 1 if any critical check fails.
# =============================================================================

set -euo pipefail

# ── Colour helpers ────────────────────────────────────────────────────────────
RED='\033[0;31m'
YEL='\033[0;33m'
GRN='\033[0;32m'
BLU='\033[0;34m'
NC='\033[0m'  # No colour

PASS=0
WARN=0
FAIL=0

ok()   { echo -e "${GRN}[PASS]${NC} $*"; ((PASS++));  }
warn() { echo -e "${YEL}[WARN]${NC} $*"; ((WARN++));  }
fail() { echo -e "${RED}[FAIL]${NC} $*"; ((FAIL++));  }
info() { echo -e "${BLU}[INFO]${NC} $*"; }

# ── Header ────────────────────────────────────────────────────────────────────
echo ""
echo -e "${BLU}╔════════════════════════════════════════════╗${NC}"
echo -e "${BLU}║   JetRacer Bring-Up Health Check           ║${NC}"
echo -e "${BLU}╚════════════════════════════════════════════╝${NC}"
echo ""

# ── 1. ROS 2 environment ──────────────────────────────────────────────────────
info "Checking ROS 2 environment..."
if command -v ros2 &>/dev/null; then
    ROS_DISTRO="${ROS_DISTRO:-unknown}"
    ok "ros2 CLI found (ROS_DISTRO=${ROS_DISTRO})"
else
    fail "ros2 CLI not found. Source /opt/ros/<distro>/setup.bash first."
fi

if [[ -n "${AMENT_PREFIX_PATH:-}" ]]; then
    ok "Workspace sourced (AMENT_PREFIX_PATH is set)"
else
    warn "AMENT_PREFIX_PATH is empty. Source install/setup.bash for overlay packages."
fi

# ── 2. Serial port (hardware driver) ─────────────────────────────────────────
info "Checking serial port..."
SERIAL_PORT="${JETRACER_SERIAL_PORT:-/dev/ttyACM0}"
if [[ -e "$SERIAL_PORT" ]]; then
    ok "Serial port $SERIAL_PORT exists"
    # Check read/write permissions
    if [[ -r "$SERIAL_PORT" && -w "$SERIAL_PORT" ]]; then
        ok "Serial port $SERIAL_PORT is readable and writable"
    else
        fail "Serial port $SERIAL_PORT permission denied. Run: sudo usermod -aG dialout \$USER && logout"
    fi
else
    warn "Serial port $SERIAL_PORT not found. Hardware driver will fail to start."
    info "  Try: ls /dev/ttyACM* /dev/ttyUSB*  to find the correct port."
fi

# ── 3. Camera / CSI pipeline ──────────────────────────────────────────────────
info "Checking CSI camera..."
if ls /dev/video* &>/dev/null 2>&1; then
    ok "Video device(s) found: $(ls /dev/video* | tr '\n' ' ')"
else
    warn "No /dev/video* found. CSI camera may still be accessible via nvarguscamerasrc."
fi

# Check that nvarguscamerasrc is available (indicates Jetson multimedia stack)
if gst-inspect-1.0 nvarguscamerasrc &>/dev/null 2>&1; then
    ok "nvarguscamerasrc GStreamer plugin is available"
else
    warn "nvarguscamerasrc not found. Ensure jetson-multimedia-api is installed."
fi

# ── 4. I2C / GPIO permissions ─────────────────────────────────────────────────
info "Checking I2C access..."
I2C_FOUND=0
for i2c_dev in /dev/i2c-*; do
    if [[ -e "$i2c_dev" ]]; then
        I2C_FOUND=1
        if [[ -r "$i2c_dev" && -w "$i2c_dev" ]]; then
            ok "I2C device $i2c_dev is accessible"
        else
            warn "I2C device $i2c_dev permission denied. Run: sudo usermod -aG i2c \$USER && logout"
        fi
    fi
done
if [[ $I2C_FOUND -eq 0 ]]; then
    warn "No /dev/i2c-* devices found."
fi

# ── 5. Battery / thermal (quick sysfs check) ──────────────────────────────────
info "Checking thermal sysfs..."
THERMAL_FOUND=0
for zone in /sys/class/thermal/thermal_zone*/temp; do
    if [[ -r "$zone" ]]; then
        ZONE_DIR=$(dirname "$zone")
        ZONE_TYPE=$(cat "$ZONE_DIR/type" 2>/dev/null || echo "unknown")
        TEMP_MIL=$(cat "$zone" 2>/dev/null || echo "0")
        TEMP_C=$(echo "scale=1; $TEMP_MIL / 1000" | bc 2>/dev/null || echo "?")
        ok "Thermal zone ${ZONE_TYPE}: ${TEMP_C}°C"
        THERMAL_FOUND=1
    fi
done
if [[ $THERMAL_FOUND -eq 0 ]]; then
    warn "No readable thermal zones found."
fi

# ── 6. ROS 2 topic availability (requires a running roscore/daemon) ───────────
info "Probing live ROS 2 topics (2-second wait)..."
if timeout 3 ros2 topic list &>/dev/null 2>&1; then
    # Check for camera topic
    if timeout 3 ros2 topic list 2>/dev/null | grep -q "csi_cam_0/image_raw"; then
        ok "Camera topic /csi_cam_0/image_raw is advertised"
    else
        warn "Camera topic not found. Is the CSI camera node running?"
    fi

    # Check for battery state
    if timeout 3 ros2 topic list 2>/dev/null | grep -q "battery_state"; then
        ok "Battery state topic is advertised"
    else
        warn "battery_state topic not found. Is jetracer_hardware running?"
    fi

    # Check for diagnostics
    if timeout 3 ros2 topic list 2>/dev/null | grep -q "/diagnostics"; then
        ok "/diagnostics topic is advertised"
    else
        warn "/diagnostics not found. Thermal monitor and hardware diagnostics may not be running."
    fi
else
    info "No live ROS 2 graph found (stack not running \u2014 topic checks skipped)."
fi

# ── 7. Disk space ─────────────────────────────────────────────────────────────
info "Checking disk space..."
FREE_KB=$(df / --output=avail | tail -1 | tr -d ' ')
FREE_MB=$((FREE_KB / 1024))
if [[ $FREE_MB -lt 500 ]]; then
    fail "Low disk space: ${FREE_MB} MB free. Bag recording and logging may fail."
elif [[ $FREE_MB -lt 2000 ]]; then
    warn "Disk space below 2 GB: ${FREE_MB} MB free."
else
    ok "Disk space: ${FREE_MB} MB free"
fi

# ── Summary ───────────────────────────────────────────────────────────────────
echo ""
echo -e "${BLU}═══════════════════════════════════════════════${NC}"
echo -e "  Results: ${GRN}${PASS} passed${NC}  ${YEL}${WARN} warnings${NC}  ${RED}${FAIL} failed${NC}"
echo -e "${BLU}═══════════════════════════════════════════════${NC}"

if [[ $FAIL -gt 0 ]]; then
    echo -e "${RED}Health check FAILED. Fix critical issues before launching.${NC}"
    exit 1
elif [[ $WARN -gt 0 ]]; then
    echo -e "${YEL}Health check passed with warnings. Review before autonomous operation.${NC}"
    exit 0
else
    echo -e "${GRN}All checks passed. Safe to launch.${NC}"
    exit 0
fi
