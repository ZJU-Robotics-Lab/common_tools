#include <iostream>                 //标准输入输出头文件
#include <fstream>
#include <pcl/io/pcd_io.h>          //I/O操作头文件
#include <pcl/io/ply_io.h>
#include <pcl/point_types.h>        //点类型定义头文件
#define BOOST_TYPEOF_EMULATION  //要加在#include <pcl/registration/icp.h>前
#include <pcl/registration/icp.h>   //ICP配准类相关头文件
#include <pcl/visualization/cloud_viewer.h>//点云查看窗口头文件
#include <pcl/visualization/pcl_visualizer.h>//可视化头文件
#include <pcl/common/transforms.h>  
#include <pcl/point_cloud.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <eigen3/Eigen/Dense>

struct Config
{
    std::string ply_gt_path;
    std::string ply_in_path;
    std::string ply_result_path;
    std::string ply_result_color_path;
    std::string ply_gt_result_path;
    int iter_num;
    double scale_init;
    double scale_step;
    int scale_iter_num;
    double distance_range;
    void show_info(void);
};

typedef pcl::PointXYZRGB PointRGBT;
typedef pcl::PointCloud<PointRGBT> PointCloudRGBT;

Config load_yaml(std::string path);
Eigen::Vector3i dis2rgb(int tmp);
void load_ply(PointCloudRGBT::Ptr &cloud, 
              std::string path);
double ICP_iter(PointCloudRGBT::Ptr cloud_gt, 
                PointCloudRGBT::Ptr cloud_in, 
                PointCloudRGBT::Ptr &cloud_out, 
                int iter, 
                pcl::KdTreeFLANN<pcl::PointXYZRGB> kdtree);
void scale_ply(PointCloudRGBT::Ptr cloud, 
               PointCloudRGBT::Ptr &cloud_out, 
               double scale);
Eigen::Vector3d get_ply_center(PointCloudRGBT::Ptr cloud);
void center_ply(PointCloudRGBT::Ptr &cloud, 
                Eigen::Vector3d center);
void show_ply(PointCloudRGBT::Ptr cloud1, 
              PointCloudRGBT::Ptr cloud2);
void calcu_error_and_color_error(PointCloudRGBT::Ptr cloud_in, 
                                 PointCloudRGBT::Ptr &cloud_out, 
                                 PointCloudRGBT::Ptr &cloud_out_colormap, 
                                 pcl::KdTreeFLANN<pcl::PointXYZRGB> kdtree);
void traverse_scale(PointCloudRGBT::Ptr cloud_str, 
                    PointCloudRGBT::Ptr cloud_gt,
                    PointCloudRGBT::Ptr &cloud_result,
                    Config config,
                    pcl::KdTreeFLANN<pcl::PointXYZRGB> kdtree);

Eigen::Vector3i dis2rgb(int tmp)
{
	Eigen::Vector3i rgb;
	if (tmp > 255)
		tmp = 255;

	if ( tmp <= 51)
	{
		rgb[2] = 255;
		rgb[1] = tmp*5;
		rgb[0] = 0;
	}
	else if (tmp <= 102)
	{
		tmp-=51;
		rgb[2] = 255-tmp*5;
		rgb[1] = 255;
		rgb[0] = 0;
	}
	else if (tmp <= 153)
	{
		tmp-=102;
		rgb[2] = 0;
		rgb[1] = 255;
		rgb[0] = tmp*5;
	}
	else if (tmp <= 204)
	{
		tmp-=153;
		rgb[2] = 0;
		rgb[1] = 255-u_char(128.0*tmp/51.0+0.5);
		rgb[0] = 255;
	}
	else
	{
		tmp-=204;
		rgb[2] = 0;
		rgb[1] = 127-u_char(127.0*tmp/51.0+0.5);
		rgb[0] = 255;
	}
	return rgb;
}

void load_ply(PointCloudRGBT::Ptr &cloud, std::string path)
{
	if (pcl::io::loadPLYFile<pcl::PointXYZRGB>(path.c_str(), *cloud) == -1)
	{
		PCL_ERROR("Couldn't read file1 \n");
	}
	std::cout << "Loaded " << cloud->size() << " points from " << path << std::endl;
}

double ICP_iter(PointCloudRGBT::Ptr cloud_gt, PointCloudRGBT::Ptr cloud_in, PointCloudRGBT::Ptr &cloud_out, int iter , pcl::KdTreeFLANN<pcl::PointXYZRGB> kdtree)
{
    PointCloudRGBT::Ptr cloud_tmp(new PointCloudRGBT);
    pcl::IterativeClosestPoint<pcl::PointXYZRGB, pcl::PointXYZRGB> icp;
	icp.setMaximumIterations(iter);    //设置最大迭代次数iterations=true
	icp.setInputCloud(cloud_in); //设置输入点云
	icp.setInputTarget(cloud_gt); //设置目标点云（输入点云进行仿射变换，得到目标点云）

	icp.align(*cloud_out);          //匹配后源点云
	if (!icp.hasConverged())//icp.hasConverged ()=1（true）输出变换矩阵的适合性评估
	{
		PCL_ERROR("\nICP has not converged.\n");
	}
    return icp.getFitnessScore();
}

void scale_ply(PointCloudRGBT::Ptr cloud, PointCloudRGBT::Ptr &cloud_out, double scale)
{
    pcl::copyPointCloud(*cloud, *cloud_out);
    for (size_t i = 0; i < cloud->points.size(); i++)
    {
        cloud_out->points[i].x = cloud->points[i].x * scale;
        cloud_out->points[i].y = cloud->points[i].y * scale;
        cloud_out->points[i].z = cloud->points[i].z * scale;
    }
}

Eigen::Vector3d get_ply_center(PointCloudRGBT::Ptr cloud)
{
    double x_center = 0;
    double y_center = 0;
    double z_center = 0;

    for (size_t i = 0; i < cloud->points.size(); i++)
    {
        x_center = (i * x_center + cloud->points[i].x) / (i+1);
        y_center = (i * y_center + cloud->points[i].y) / (i+1);
        z_center = (i * z_center + cloud->points[i].z) / (i+1);
    }
    
    Eigen::Vector3d Center(x_center, y_center, z_center);
    std::cout << " Ply Center = " << x_center << " " << y_center << " " << z_center << " " << std::endl;
    return Center;
}

void center_ply(PointCloudRGBT::Ptr &cloud, Eigen::Vector3d center)
{
    for (size_t i = 0; i < cloud->points.size(); i++)
    {
        cloud->points[i].x = cloud->points[i].x - center[0];
        cloud->points[i].y = cloud->points[i].y - center[1];
        cloud->points[i].z = cloud->points[i].z - center[2];
    }
}

Config load_yaml(std::string path)
{
    std::ifstream inf;
    inf.open(path.c_str());
    Config config;
    if(!inf.is_open())
    {
        std::cout << "File :" << path << " Open Failure!" << std::endl;
    }
    while (!inf.eof())
    {
        std::string s;
        std::stringstream ss;
        inf >> s;
        if (s == "ply_src_path:")
        {
            inf >> s;
            ss << s;
            ss >> config.ply_in_path;
        }
        if (s == "ply_tar_path:")
        {
            inf >> s;
            ss << s;
            ss >> config.ply_gt_path;
        }
        if (s == "icp_iter_num:")
        {
            inf >> s;
            ss << s;
            ss >> config.iter_num;
        }
        if (s == "scale_init:")
        {
            inf >> s;
            ss << s;
            ss >> config.scale_init;
        }
        if (s == "scale_step:")
        {
            inf >> s;
            ss << s;
            ss >> config.scale_step;
        }
        if (s == "scale_iter_num:")
        {
            inf >> s;
            ss << s;
            ss >> config.scale_iter_num;
        }
        if (s == "distance_range:")
        {
            inf >> s;
            ss << s;
            ss >> config.distance_range;
        }
    }
    std::string ply_result_path(config.ply_in_path, 0, config.ply_in_path.size()-4);
    ply_result_path += "_icp_result.ply"; 
    std::string ply_result_color_path(config.ply_in_path, 0, config.ply_in_path.size()-4);
    ply_result_color_path += "_colormap.ply";
    std::string ply_gt_result_path(config.ply_gt_path, 0, config.ply_gt_path.size()-4);
    ply_gt_result_path += "_center.ply";
    config.ply_gt_result_path = ply_gt_result_path;
    config.ply_result_color_path = ply_result_color_path;
    config.ply_result_path = ply_result_path;
    config.show_info();
    return config;
}

void Config::show_info(void){
    std::cout << std::endl;
    std::cout << std::endl;
    std::cout << "******Config INFO*********" << std::endl;
    std::cout << "*ply_src_path : " << ply_in_path << std::endl;
    std::cout << "*ply_tar_path : " << ply_gt_path << std::endl;
    std::cout << "*ply_tar_center_path : " << ply_gt_result_path << std::endl;
    std::cout << "*ply_result_colormap_path : " << ply_result_color_path << std::endl;
    std::cout << "*ply_result_path : " << ply_result_path << std::endl;
    std::cout << "*icp_iter_num : " << iter_num << std::endl;
    std::cout << "*scale_init : " << scale_init << std::endl;
    std::cout << "*scale_step : " << scale_step << std::endl;
    std::cout << "*scale_iter_num : " << scale_iter_num << std::endl;
    std::cout << "*distance_range : " << distance_range << std::endl;
    std::cout << "************************" << std::endl;
    std::cout << std::endl;
    std::cout << std::endl;
}

void show_ply(PointCloudRGBT::Ptr cloud1, PointCloudRGBT::Ptr cloud2)
{
    //可视化
	pcl::visualization::PCLVisualizer viewer("ICP Result");

	viewer.addPointCloud(cloud1, "cloud1");
	viewer.addPointCloud(cloud2, "cloud2");

	// 加入文本的描述在各自的视口界面
	//在指定视口viewport=v1添加字符串“white 。。。”其中"icp_info_1"是添加字符串的ID标志，（10，15）为坐标16为字符大小 后面分别是RGB值
	viewer.addText("White: Original point cloud\nRed: ICP aligned point cloud", 10, 15);

	// 设置背景颜色 
	viewer.setBackgroundColor(0,0,0);
	// 设置相机的坐标和方向
	viewer.setCameraPosition(-3.68332, 2.94092, 5.71266, 0.289847, 0.921947, -0.256907, 0);
	viewer.setSize(1280, 1024);  // 可视化窗口的大小	

	//显示
	while (!viewer.wasStopped())
	{
		viewer.spinOnce();
	}
}

void calcu_error_and_color_error(PointCloudRGBT::Ptr cloud_in, PointCloudRGBT::Ptr &cloud_out, PointCloudRGBT::Ptr &cloud_out_coloramp, pcl::KdTreeFLANN<pcl::PointXYZRGB> kdtree)
{
    int K = 1;
    double dis_sum = 0;
	for (size_t i = 0; i < cloud_in->points.size(); i++)
	{
		pcl::PointXYZRGB p;
		p = cloud_in->points[i];
		std::vector<int> pointIdxNKNSearch(K);
		std::vector<float> pointIdxNKNSquaredDistance(K);
		kdtree.nearestKSearch(p, K, pointIdxNKNSearch, pointIdxNKNSquaredDistance);
        double distance = std::sqrt(pointIdxNKNSquaredDistance[0]);
        // 20 mm以上的误差点忽略不计
        if (distance > 5)
            continue;
        
        int dis_255 = int(distance*255/5);
        if(dis_255 > 255) 
            dis_255 = 255;
        Eigen::Vector3i rgb = dis2rgb(dis_255);
        cloud_out->points.push_back(p);
        p.r = rgb[0];
        p.g = rgb[1];
        p.b = rgb[2];
        cloud_out_coloramp->points.push_back(p);
        dis_sum += distance;
	}

    double dis_mean = dis_sum / cloud_out->points.size();
    std::cout << "Dis Mean = " << dis_mean << std::endl;
    std::cout << "Final Result Point Num = " << cloud_out->points.size() << std::endl;

}

void traverse_scale(PointCloudRGBT::Ptr cloud_str, 
                    PointCloudRGBT::Ptr cloud_tar, 
                    PointCloudRGBT::Ptr &cloud_result, 
                    Config config,
                    pcl::KdTreeFLANN<pcl::PointXYZRGB> kdtree)
{
    double scale_step = config.scale_step;
    double scale= config.scale_init;
    int scale_iter = config.scale_iter_num;
    double min_score = 1000000000;
    double best_scale = scale;
    PointCloudRGBT::Ptr cloud_scale(new PointCloudRGBT);
    for (size_t i = 0; i < scale_iter; i++)
    {
        scale= scale + scale_step;
        scale_ply(cloud_str, cloud_scale, scale);
        double icp_score = ICP_iter(cloud_tar, cloud_scale, cloud_scale, config.iter_num, kdtree);
        std::cout << " Scale " << scale << " ; score = " << icp_score <<  std::endl;
        if (icp_score < min_score)
        {
            min_score = icp_score;
            best_scale = scale;
            pcl::copyPointCloud(*cloud_scale, *cloud_result);
        }
    }
    
    std::cout << " Traverse Scale Finish: " << std::endl;
    std::cout << " Best Scale = " << best_scale << " ; score = " << min_score << std::endl;
}






int main(int argc, char const *argv[])
{
    // 点云指针初始化
    PointCloudRGBT::Ptr cloud_gt(new PointCloudRGBT);
    PointCloudRGBT::Ptr cloud_in(new PointCloudRGBT);
    PointCloudRGBT::Ptr cloud_result(new PointCloudRGBT);

    // 加载配置参数
    Config config = load_yaml("../config/icp.yaml");

    // 加载点云并中心化
    load_ply(cloud_in, config.ply_in_path);
    load_ply(cloud_gt, config.ply_gt_path);
    Eigen::Vector3d center_gt = get_ply_center(cloud_gt);
    center_ply(cloud_in, center_gt);
    center_ply(cloud_gt, center_gt);

    // 为真值点云构建Kdtree
    pcl::KdTreeFLANN<pcl::PointXYZRGB> kdtree;
    kdtree.setInputCloud(cloud_gt);

    // 遍历Scale计算ICP并输出Score
    PointCloudRGBT::Ptr cloud_icp(new PointCloudRGBT);
    traverse_scale(cloud_in, cloud_gt, cloud_icp, config, kdtree);

    // 在上述中选择最小误差ICP结果，计算误差，剔除外点并根据距离着色
    PointCloudRGBT::Ptr cloud_final(new PointCloudRGBT);
    PointCloudRGBT::Ptr cloud_final_color(new PointCloudRGBT);
    calcu_error_and_color_error(cloud_icp, cloud_final, cloud_final_color, kdtree);

    // 保存点云
    pcl::io::savePLYFile(config.ply_result_path, *cloud_final);
    pcl::io::savePLYFile(config.ply_gt_result_path, *cloud_gt);
    pcl::io::savePLYFile(config.ply_result_color_path, *cloud_final_color);

    // 点云可视化
    show_ply(cloud_final_color, cloud_gt);

    return 0;  
}
