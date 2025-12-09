#!/bin/bash
RULES=~/uchile_robotics/uchile_system/jaime/core/config/udev_scripts/99-jaime.rules

echo "Installing udev rules from $RULES"
sudo cp "$RULES" /etc/udev/rules.d/
sudo udevadm control --reload-rules && sudo udevadm trigger

echo "Done!"
