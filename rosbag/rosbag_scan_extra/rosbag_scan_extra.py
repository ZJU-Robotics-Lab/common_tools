import os
import sys
import rospy
import rosbag
import argparse
from sensor_msgs import point_cloud2
from sensor_msgs.msg import LaserScan
from laser_geometry import LaserProjection

def parse_args():
    parser = argparse.ArgumentParser()
    parser.add_argument("--rosbag", type=str, required=True)
    parser.add_argument("--topic", type=str, default='/scan')
    args = parser.parse_args()
    return args

def mkdir(path):
    if os.path.exists(path):
        print('Path : ', path ,' already exists!')
    else:
        os.mkdir(path)

def write_pcd(path, pointcloud):
    with open(path, 'w') as f:
        f.write('# .PCD v0.7 - Point Cloud Data file format\n')
        f.write('VERSION 0.7\n')
        f.write('FIELDS x y z\n')
        f.write('SIZE 4 4 4\n')
        f.write('TYPE F F F\n')
        f.write('COUNT 1 1 1\n')
        f.write('WIDTH ' + str(len(pointcloud)) + '\n')
        f.write('HEIGHT 1\n')
        f.write('VIEWPOINT 0 0 0 1 0 0 0\n')
        f.write('POINTS ' + str(len(pointcloud)) + '\n' )
        f.write('DATA ascii\n')
        for p in pointcloud:
            f.write(str(p[0]) + ' ' + str(p[1]) + ' ' + str(p[2]) + '\n')
    f.close()

if __name__ == "__main__":
    args = parse_args()
    
    rosbag_path = args.rosbag
    rosbag_pcd_path_root = rosbag_path[0:-4]
    mkdir(rosbag_pcd_path_root)
    
    laserProj = LaserProjection()
    scan_num = 0
    print "Start Process ", rosbag_path 
    scan_list = []
    with rosbag.Bag(rosbag_path, 'r') as bag:
        for topic,msg,t in bag.read_messages():
            if topic == args.topic:
                scan_num += 1
                cloud = laserProj.projectLaser(msg)
                points = []
                for data in point_cloud2.read_points(cloud, skip_nans=True):
                    points.append(data[0:3])
                pcd_name = str(t) + '.pcd'
                pcd_path = os.path.join(rosbag_pcd_path_root,pcd_name)
                scan_list.append(pcd_name)
                write_pcd(pcd_path, points)
    with open(rosbag_pcd_path_root + '.txt', 'w') as f:
        for scan in scan_list:
            f.write(scan + '\n')
        f.close()
    print 'Process Finish : Total ', scan_num, ' scans!' 
    print 'Pcd File Save in :', rosbag_pcd_path_root 
