#!/bin/bash
RULES=~/uchile_robotics/uchile_system/nero/core/config/udev_scripts/99-nero.rules

echo "Installing udev rules from $RULES"
sudo cp "$RULES" /etc/udev/rules.d/
sudo udevadm control --reload-rules && sudo udevadm trigger --subsystem-match=net --action=add

echo "Done! (if the CAN module was already plugged in, unplug and replug it)"
