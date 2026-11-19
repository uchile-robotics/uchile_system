#!/bin/bash

echo "Deleting nero udev rules"
sudo rm -f /etc/udev/rules.d/99-nero.rules
sudo udevadm control --reload-rules && sudo udevadm trigger
echo "Done!"
