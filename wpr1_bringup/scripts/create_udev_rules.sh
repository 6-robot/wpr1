#!/bin/bash

echo "***************"
echo "remap the device serial ports(ttyUSBX) to wpr1 device"
echo "start copy wpr1.rules to  /etc/udev/rules.d/"
sudo cp `rospack find wpr1_bringup`/scripts/wpr1.rules  /etc/udev/rules.d
echo "start copy 90-kinect2.rules to  /etc/udev/rules.d/"
sudo cp `rospack find wpr1_bringup`/scripts/90-kinect2.rules  /etc/udev/rules.d
echo "Restarting udev"
sudo service udev reload
sudo service udev restart
echo "finish"
echo "***************"
