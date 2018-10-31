#!/bin/bash

hander()
{
    sleep 1
}

trap hander SIGINT

TIME=$(date +%Y-%m-%d-%H-%M-%S)
CLOUD="/velodyne_packets"
ODOM="/odom"
IMU="/imu/data"
REALSENSE="/camera/color/image_raw/compressed /camera/color/camera_info"

echo $TIME &
echo $CLOUD &
echo $ODOM &
echo $IMU &
echo $REALSENSE &

/opt/ros/kinetic/bin/rosbag record $CLOUD $ODOM $IMU $REALSENSE -O /home/amsl/$TIME.bag
