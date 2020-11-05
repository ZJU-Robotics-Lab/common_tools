import pcl
import numpy as np

if __name__ == "__main__":
    
    ply_raw = pcl.load_XYZRGB('/home/qk/Documents/Mvs_dataset/stone_data/pointcloud/fused.ply')
    print(ply_raw.size)
    

    ply_raw_np = np.array(ply_raw)
    ply_raw_np_25 = ply_raw_np
    ply_raw_np_25[:,0:3] = ply_raw_np[:,0:3] *25

    x_mean = np.sum(ply_raw_np_25[:, 0])/ply_raw.size
    y_mean = np.sum(ply_raw_np_25[:, 1])/ply_raw.size
    z_mean = np.sum(ply_raw_np_25[:, 2])/ply_raw.size
    print('center [x,y,z] = ', x_mean, y_mean, z_mean)
    print(ply_raw_np[0])
    ply_raw_np_25[:, 0] = ply_raw_np_25[:, 0] - x_mean
    ply_raw_np_25[:, 1] = ply_raw_np_25[:, 1] - y_mean
    ply_raw_np_25[:, 2] = ply_raw_np_25[:, 2] - z_mean

    ply_25 = pcl.PointCloud_PointXYZRGB()
    ply_25.from_array(ply_raw_np_25)
    print(ply_raw_np_25[0])
    # print(ply_25)
    pcl.save(ply_25, '/home/qk/Documents/Mvs_dataset/stone_data/pointcloud/fused_25_rgb.ply')