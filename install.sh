#!/usr/bin/env bash

# ============================================================
# WT901C485 ROS 2 Driver - Dependency Installer
# Target OS : Ubuntu 22.04
# Target ROS: ROS 2 Humble
# ============================================================

set -e

echo "=============================================="
echo " WT901C485 ROS 2 Driver - Dependency Installer "
echo "=============================================="

# ------------------------------
# 1. Check OS
# ------------------------------
if ! grep -q "Ubuntu 22.04" /etc/os-release; then
  echo "[ERROR] This script is intended for Ubuntu 22.04 only."
  exit 1
fi

# ------------------------------
# 2. Check ROS 2 Humble
# ------------------------------
if [ ! -d "/opt/ros/humble" ]; then
  echo "[ERROR] ROS 2 Humble is not installed."
  echo "Install ROS 2 Humble before running this script."
  echo "https://docs.ros.org/en/humble/Installation.html"
  exit 1
fi

echo "[OK] Ubuntu 22.04 detected"
echo "[OK] ROS 2 Humble detected"

# ------------------------------
# 3. Update system
# ------------------------------
echo "[INFO] Updating system packages..."
sudo apt update

# ------------------------------
# 4. Install system dependencies
# ------------------------------
echo "[INFO] Installing system dependencies..."

sudo apt install -y \
  python3-pip \
  python3-serial \
  python3-colcon-common-extensions \
  python3-rosdep \
  udev \
  build-essential \
  git

# ------------------------------
# 5. Install kernel modules (USB-RS485 / CH340)
# ------------------------------
echo "[INFO] Installing kernel extra modules (CH340 support)..."

sudo apt install -y linux-modules-extra-$(uname -r)

# ------------------------------
# 6. Remove conflicting services (BRLTTY)
# ------------------------------
if dpkg -l | grep -q brltty; then
  echo "[INFO] Removing brltty (conflicts with USB serial devices)..."
  sudo apt purge -y brltty
fi

# ------------------------------
# 7. Python dependencies
# ------------------------------
echo "[INFO] Installing Python dependencies..."

pip3 install --upgrade pip
pip3 install pyserial

# ------------------------------
# 8. ROS dependency initialization
# ------------------------------
echo "[INFO] Initializing rosdep..."

if [ ! -f "/etc/ros/rosdep/sources.list.d/20-default.list" ]; then
  sudo rosdep init
fi
rosdep update

# ------------------------------
# 9. udev rule for WT901C (optional but recommended)
# ------------------------------
UDEV_RULE_FILE="/etc/udev/rules.d/99-wt901c.rules"

if [ ! -f "$UDEV_RULE_FILE" ]; then
  echo "[INFO] Installing udev rule for WT901C IMU..."

  sudo tee $UDEV_RULE_FILE > /dev/null <<EOF
SUBSYSTEM=="tty", ATTRS{idVendor}=="1a86", ATTRS{idProduct}=="7523", SYMLINK+="wt901c"
EOF

  sudo udevadm control --reload-rules
  sudo udevadm trigger
else
  echo "[INFO] udev rule already exists"
fi

# ------------------------------
# 10. Permissions
# ------------------------------
echo "[INFO] Adding user to dialout group..."

sudo usermod -a -G dialout $USER

# ------------------------------
# 11. Final message
# ------------------------------
echo ""
echo "=============================================="
echo " Installation complete"
echo "=============================================="
echo ""
echo "IMPORTANT:"
echo "1. Reboot your system:"
echo "   sudo reboot"
echo ""
echo "2. After reboot:"
echo "   source /opt/ros/humble/setup.bash"
echo ""
echo "3. Build your workspace:"
echo "   colcon build"
echo ""
echo "4. Run the driver:"
echo "   ros2 run wt901c_imu wt901c_node"
echo ""
echo "=============================================="
