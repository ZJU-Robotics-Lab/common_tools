- ### rosbag相关

  - #### **rosbag提取image(imu/pointcloud)**

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

  - #### rosbag合并

    ```
    bash rosbag_merge/run_merge.bash
    ```

    


