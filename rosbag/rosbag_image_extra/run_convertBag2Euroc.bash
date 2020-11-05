#!/usr/bin/env bash
echo "run!!!"
srcPath=/media/qk/xinjiajuan/1029/exposure_5000ms 
destPath=/media/qk/xinjiajuan/1029/exposure_5000ms 
cam0_topic=/read_image
cam1_topic=/
imu_topic=/
lidar_topic=/
echo "srcPath: ${srcPath}"
echo "destPath: ${destPath}"
echo "cam0_topic: ${cam0_topic}"
echo "cam1_topic: ${cam1_topic}"
echo "imu_topic: ${imu_topic}"
echo "lidar_topic: ${lidar_topic}"
python convertBag2Euroc.py ${srcPath} ${destPath} ${cam0_topic} ${cam1_topic} ${imu_topic} ${lidar_topic}
