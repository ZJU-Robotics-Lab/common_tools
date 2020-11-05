

- ### pointcloud相关

  - #### ICP配准并根据误差距离着色

    [本脚本默认输入的点云格式为PointcloudXYZRGB]

    配置参数在/cpp/config/icp.yaml中修改：

    ```yaml
    ply_src_path: /path_to_src_pointcloud
    ply_tar_path: /path_to_tar_pointcloud
    icp_iter_num: /ICP迭代次数
    scale_init: /初始尺度
    scale_step: /尺度迭代步长
    scale_iter_num: /尺度迭代次数
    distance_range: /误差着色最远距离
    ```

    本可执行文件初衷是调整尺度以获得最适合点云的尺度，并着色。若无需调整尺度，则按 scale_init = 1, scale_step =0, scale_iter_num = 1 进行配置即可。

    其中distance_range代表着: 1.最大着色误差，及误差>=distance_range的也将按此颜色着色; 2.在统计最终平均距离误差时， >distance_range的点将被剔除。

    **注意**：最终结果将保存在三个点云中，分别代表

    ​	ply_tar_center_path: 目标点云按中心平移后的点云

    ​	ply_result_colormap_path: ICP结果平移并着色后点云

    ​	ply_result_path: ICP结果平移后点云

  - #### **PCD转PLY**

    直接修改py文件中的pcd_path运行即可

    

