#!/usr/bin/python
#coding:utf-8

# Extract images from a bag file.
import sys
import os
import roslib;  
import rosbag
import rospy
import cv2
import datetime
from sensor_msgs.msg import Image
from sensor_msgs.msg import Imu
from sensor_msgs import point_cloud2 as pc2
from sensor_msgs.msg import PointCloud2
from cv_bridge import CvBridge
from cv_bridge import CvBridgeError

def Points2Pcd(lidar_msg, lidar_dir_path, lidar_name):
	# 存放路径
    PCD_FILE_PATH=lidar_dir_path + "/data/" + lidar_name
    if os.path.exists(PCD_FILE_PATH):
    	os.remove(PCD_FILE_PATH)
    lidar_points_gen = pc2.read_points(lidar_msg, field_names=("x", "y", "z", "intensity", "ring"), skip_nans=True)
    # 写文件句柄
    handle = open(PCD_FILE_PATH, 'a')

    # pcd头部
    # PointCloud2数据格式详见： http://docs.ros.org/jade/api/sensor_msgs/html/msg/PointCloud2.html
    # fields中的dataType值含义: http://docs.ros.org/jade/api/sensor_msgs/html/msg/PointField.html
    handle.write('# .PCD v0.7 - Point Cloud Data file format\nVERSION 0.7\nFIELDS x y z intensity ring\nSIZE 4 4 4 4 4\nTYPE F F F F I\nCOUNT 1 1 1 1 1')
    string = '\nWIDTH ' + str(lidar_msg.width)
    handle.write(string)
    handle.write('\nHEIGHT 1\nVIEWPOINT 0 0 0 1 0 0 0')
    string = '\nPOINTS ' + str(lidar_msg.width * lidar_msg.height)
    handle.write(string)
    handle.write('\nDATA ascii')

    # 依次写入点
    for p in lidar_points_gen:
        points_string = '\n' + str(p[0]) + ' ' + str(p[1]) + ' ' + str(p[2]) + ' '+ str(p[3]) + ' ' + str(p[4])
        handle.write(points_string)
    handle.close()

def ExtractImg(bridge, img_msg, cam_newLines, image_dir_path):
    try:
        cv_image = bridge.imgmsg_to_cv2(img_msg, "mono8")
    except CvBridgeError as e:
        print(e)
    timestr = img_msg.header.stamp.to_nsec()
    img_name = str(timestr) + ".png"
    cam_newLines.append('%s,%s\n' % (str(timestr), img_name))
    img_save_path = image_dir_path + "/data/" + img_name
    cv2.imwrite(img_save_path, cv_image)

def ExtractImu(imu_msg, imu_new_line):
    timestr = imu_msg.header.stamp.to_nsec()
    linear_acceleration_x = imu_msg.linear_acceleration.x
    linear_acceleration_y = imu_msg.linear_acceleration.y
    linear_acceleration_z = imu_msg.linear_acceleration.z
    angular_velocity_x = imu_msg.angular_velocity.x
    angular_velocity_y = imu_msg.angular_velocity.y
    angular_velocity_z = imu_msg.angular_velocity.z

    imu_new_line.append('%s,%s,%s,%s,%s,%s,%s\n' % (str(timestr), str(angular_velocity_x), str(angular_velocity_y), str(angular_velocity_z), \
                                                str(linear_acceleration_x), str(linear_acceleration_y), str(linear_acceleration_z)))

def ExtractLidar(lidar_msg, lidar_newLines, lidar_dir_path):
    lidar_timestamp = lidar_msg.header.stamp.to_nsec()
    lidar_name = str(lidar_timestamp) + ".pcd"
    lidar_newLines.append('%s,%s\n' % (str(lidar_timestamp), lidar_name))

    Points2Pcd(lidar_msg, lidar_dir_path, lidar_name)

def RosbagExtract(inDir, outDir, cam0_topic, cam1_topic, imu_topic, lidar_topic):
    bridge = CvBridge()
    cam0_newLines = []
    cam1_newLines = []
    imu_newLines = []
    lidar_newLines = []
    has_cam0_data = False
    has_cam1_data = False
    has_imu_data = False
    has_lidar_data = False
    image0_dir_path = os.path.join(outDir, 'mav0', 'cam0')
    image1_dir_path = os.path.join(outDir, 'mav0', 'cam1')
    imu_dir_path = os.path.join(outDir, 'mav0', 'imu0')
    lidar_dir_path = os.path.join(outDir, 'mav0', 'pointcloud0')

    with rosbag.Bag(inDir, 'r') as bag: 
        for topic,msg,t in bag.read_messages():
            if topic == cam0_topic:
                has_cam0_data = True
                ExtractImg(bridge, msg, cam0_newLines, image0_dir_path)
            if topic == cam1_topic:
                has_cam1_data = True
                ExtractImg(bridge, msg, cam1_newLines, image1_dir_path)
            if topic == imu_topic:
                has_imu_data = True
                ExtractImu(msg, imu_newLines)
            if topic == lidar_topic:
                has_lidar_data = True
                ExtractLidar(msg, lidar_newLines, lidar_dir_path)
    if has_cam0_data:
        cam0_data_filename = image0_dir_path + "/data.csv"
        with open(cam0_data_filename, 'w') as outCsv:
            outCsv.writelines(cam0_newLines)
    if has_cam1_data:
        cam1_data_filename = image1_dir_path + "/data.csv"
        with open(cam1_data_filename, 'w') as outCsv:
            outCsv.writelines(cam1_newLines)
    if has_imu_data:
        imu_data_filename = imu_dir_path + "/data.csv"
        with open(imu_data_filename, 'w') as outCsv:
            outCsv.writelines(imu_newLines)
    if has_lidar_data:
        lidar_data_filename = lidar_dir_path + "/data.csv"
        with open(lidar_data_filename, 'w') as outCsv:
            outCsv.writelines(lidar_newLines)

def convert(inDir, outDir, cam0_topic, cam1_topic, imu_topic, lidar_topic):
    print (datetime.datetime.now(), 'create...')
    if os.path.exists(outDir):
        print ('Error!输出目录 %s 已存在'% outDir)
        return

    os.makedirs(outDir)
    os.mkdir(os.path.join(outDir, 'mav0'))
    os.mkdir(os.path.join(outDir, 'mav0', 'cam0'))
    os.mkdir(os.path.join(outDir, 'mav0', 'cam0', 'data'))
    os.mkdir(os.path.join(outDir, 'mav0', 'cam1'))
    os.mkdir(os.path.join(outDir, 'mav0', 'cam1', 'data'))
    os.mkdir(os.path.join(outDir, 'mav0', 'imu0'))
    os.mkdir(os.path.join(outDir, 'mav0', 'pointcloud0'))
    os.mkdir(os.path.join(outDir, 'mav0', 'pointcloud0', 'data'))
    RosbagExtract(inDir,outDir,cam0_topic,cam1_topic,imu_topic,lidar_topic)
    print ('Done!')


if __name__ == '__main__':
    if len(sys.argv) < 7:
        print ('参数不正确!\n参考示例：python convertBag.py srcPath destPath cam0_topic, cam1_topic, imu_topic, lidar_topic')
        exit()
    else:
        srcPath = sys.argv[1]
        destPath = sys.argv[2]
        cam0_topic = sys.argv[3]
        cam1_topic = sys.argv[4]
        imu_topic = sys.argv[5]
        lidar_topic = sys.argv[6]

    for fileName in os.listdir(srcPath):
        if os.path.splitext(fileName)[1] == '.bag':
            bagname = os.path.splitext(fileName)[0]
            print ('\n%s---------' % fileName)
            print ('\n%s---------' % bagname)
            convert(os.path.join(srcPath, fileName), os.path.join(destPath, bagname), cam0_topic, cam1_topic, imu_topic, lidar_topic)
