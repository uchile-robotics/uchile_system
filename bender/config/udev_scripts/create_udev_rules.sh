#!/bin/bash

echo "remapping rplidar and pioneer ports\n"
echo "check the ports are correctly assigned using the command 'ls -l /dev | grep ttyUSB'\n"
echo "start copy rplidar.rules to /etc/udev/rules.d/"
sudo cp ~/uchile_robotics/uchile_system/bender/config/udev_scripts/rplidar.rules  /etc/udev/rules.d
sudo cp ~/uchile_robotics/uchile_system/bender/config/udev_scripts/pioneer.rules  /etc/udev/rules.d
sudo cp ~/uchile_robotics/uchile_system/bender/config/udev_scripts/joy.rules /etc/udev/rules.d
echo -e "\nRestarting udev\n"
sudo service udev reload
sudo service udev restart
sudo udevadm control --reload && sudo udevadm trigger
echo "finish"
