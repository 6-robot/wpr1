#!/bin/bash

echo "***************"
echo "delete the remap device serial ports of wpr1"
sudo rm   /etc/udev/rules.d/wpr1.rules
echo "Restarting udev"
sudo service udev reload
sudo service udev restart
echo "finish"
echo "***************"
