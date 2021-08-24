#!/bin/bash
sudo chmod a+rw /dev/ttyACM0
rosrun rosserial_python serial_node.py _baud:=57600 _port:=/dev/ttyACM0