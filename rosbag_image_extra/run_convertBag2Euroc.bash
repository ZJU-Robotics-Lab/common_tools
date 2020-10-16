#!/usr/bin/env bash
echo "run!!!"
srcPath=/home/qk/Desktop/1016/new_colmap/translation
destPath=/home/qk/Desktop/1016/new_colmap/translation
cam0_topic=/camera/image_raw
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
