#!/bin/bash
RULES=~/uchile_robotics/uchile_system/bender/core/config/udev_scripts/99-bender.rules

echo "Installing udev rules from $RULES"
sudo cp "$RULES" /etc/udev/rules.d/
sudo udevadm control --reload-rules && sudo udevadm trigger

echo "Done!"
