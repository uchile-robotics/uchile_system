#!/bin/bash

echo "Deleting the remappings of all bender devices"
echo "sudo rm /etc/udev/rules.d/rplidar.rules"
echo "sudo rm /etc/udev/rules.d/pioneer.rules"
echo "sudo rm /etc/udev/rules.d/joy.rules"
echo "sudo rm /etc/udev/rules.d/realsense.rules"
sudo rm /etc/udev/rules.d/rplidar.rules
sudo rm /etc/udev/rules.d/pioneer.rules
sudo rm /etc/udev/rules.d/joy.rules
sudo rm /etc/udev/rules.d/realsense.rules
echo ""
echo "Restarting udev"
echo ""
sudo service udev reload
sudo service udev restart
echo "finish delete"
