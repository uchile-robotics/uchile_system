#!/bin/bash

echo "Deleting the remappings of rplidar and the pioneer"
echo "sudo rm /etc/udev/rules.d/rplidar.rules"
sudo rm /etc/udev/rules.d/rplidar.rules
sudo rm /etc/udev/rules/pioneer.rules
sudo rm /etc/udev/rules/joy.rules
echo ""
echo "Restarting udev"
echo ""
sudo service udev reload
sudo service udev restart
echo "finish delete"
