#!/bin/bash

echo "Deleting jaime udev rules"
sudo rm -f /etc/udev/rules.d/99-jaime.rules
sudo udevadm control --reload-rules && sudo udevadm trigger
echo "Done!"
