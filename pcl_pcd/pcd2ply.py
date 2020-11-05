import pcl

if __name__ == "__main__":
    pcd_path = '/home/qk/Desktop/monolidar_data/1023/map_result/000066/cloud.pcd'
    cloud = pcl.load(pcd_path)
    pcl.save(cloud, pcd_path[:-4] + '.ply')
    
