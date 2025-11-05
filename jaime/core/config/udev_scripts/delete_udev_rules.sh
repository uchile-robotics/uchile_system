#!/bin/bash

echo "Deleting bender udev rules"
sudo rm -f /etc/udev/rules.d/99-bender.rules
sudo udevadm control --reload-rules && sudo udevadm trigger
echo "Done!"
