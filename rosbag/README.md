- ### rosbag相关

  - ### **rosbag提取image(imu/pointcloud)**

    ```shell
    bash rosbag_image_extra/run_converBag2Euroc.bash
    ```

    **依赖**：ros

    **使用说明**：

    ```
    srcPath=/home/qk/Desktop/1016/new_colmap/translation
    destPath=/home/qk/Desktop/1016/new_colmap/translation
    cam0_topic=/camera/image_raw
    cam1_topic=/
    imu_topic=/
    lidar_topic=/
    ```

    ​	在bash文件中，修改srcPath为rosbag所在路径，destPath为rosbag解析数据输出路径。bash中的topic名称则代表需要解析的image等topic名称。支持最多一次性解析两个相机、一个激光和一个IMU。输出路径的文件夹名称与rosbag名称相同，文件格式同Euroc。

  - ### **rosbag提取image(imu/pointcloud)**

    ```shell
    bash rosbag_scan_extra/run_rosbag_scan_extra.sh
    ```

    **依赖**：ros
  
    **使用说明：**
  
    ```
    python rosbag_scan_extra.py \
    --rosbag /home/qk/Desktop/2Dlaser/2020-11-09-23-16-28.bag \
    --topic /scan
    ```
  
    rosbag：bag路径
  
    topic：2D scan topic名称
  
    注：本脚本将rosbag中的/scan topic中的2D激光雷达转换为3D点云，并输出保存为pcd格式文件，输出路径文件名与rosbag名称相同。
  
  - #### rosbag合并
  
    ```
    bash rosbag_merge/run_merge.bash
    ```
  
    


