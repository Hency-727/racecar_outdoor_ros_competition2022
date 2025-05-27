#!/bin/bash
sudo cp ./car.rules /etc/udev/rules.d
sudo cp ./laser.rules /etc/udev/rules.d
sudo cp ./imu.rules /etc/udev/rules.d
sudo cp ./wheel_odom.rules /etc/udev/rules.d
#
exit 0
