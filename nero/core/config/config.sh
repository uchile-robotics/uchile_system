#!/bin/bash
echo "\n=============== CONFIG FOR NERO CORE ==============="

CORE_CONFIG_PATH="$HOME/uchile_robotics/uchile_system/nero/core/config"

# gs_usb is the kernel driver of the AgileX USB-CAN module (in mainline kernel).
# Load it now and at every boot.
sudo modprobe gs_usb
echo gs_usb | sudo tee /etc/modules-load.d/nero-gs_usb.conf > /dev/null

# Host tools used by can_scripts/ (ip, ethtool, candump)
if ! command -v ethtool &>/dev/null || ! command -v candump &>/dev/null; then
    sudo apt-get update && sudo apt-get install -y can-utils ethtool iproute2
fi

chmod +x "$CORE_CONFIG_PATH"/can_scripts/*.sh "$CORE_CONFIG_PATH"/udev_scripts/*.sh
source "$CORE_CONFIG_PATH/udev_scripts/create_udev_rules.sh"

echo "\n=============== NERO CORE CONFIG DONE! ==============="
