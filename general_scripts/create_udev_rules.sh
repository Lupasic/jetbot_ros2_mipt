#!/bin/bash

echo "remap the device serial port(ttyUSBX) to  ttyLIDAR for rplidar and ttyMOTOR for motors"
echo "start copy 99-usb-jetbot-lidar-motor.rules to  /etc/udev/rules.d/"
sudo cp ./99-usb-jetbot-lidar-motor.rules  /etc/udev/rules.d
echo " "
echo "Restarting udev"
echo ""
sudo service udev reload
sudo service udev restart
sudo udevadm control --reload && sudo udevadm trigger
echo "finish"
