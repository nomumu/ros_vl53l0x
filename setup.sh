#!/bin/sh

git clone https://github.com/pololu/vl53l0x-arduino.git

cp ./vl53l0x-arduino/VL53L0X.cpp ./src/
cp ./vl53l0x-arduino/VL53L0X.h ./include/ros_vl53l0x/

patch -u ./src/VL53L0X.cpp < ./patch/cpp.patch
patch -u ./include/ros_vl53l0x/VL53L0X.h < ./patch/header.patch

