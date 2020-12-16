#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>
#include <dirent.h>
#include <iostream>
#include <string>
#include <vector>

std::vector<std::string> sort_string(std::vector<std::string> in_array)
{
	std::vector<std::string> str_array;
    std::vector<std::string> out_array;
	int i,j = 0;
	for (int i = 0; i < in_array.size(); i++)
	{ 
		str_array.push_back(in_array[i]);
	}
	sort(str_array.begin(), str_array.end());
	std::vector<std::string>::iterator st;

	for (st = str_array.begin(); st != str_array.end(); st++)
	{
        out_array.push_back(*st);
	}
    return out_array;
}

std::vector<std::string> get_file_dir(std::string path)
{
    DIR *dp;
    struct dirent *dirp;

    if((dp = opendir(path.c_str())) == NULL)
    {
        std::cout << "Can't open " << path << std::endl;
    }
    int num = 0;

    std::vector<std::string> dirs;
    std::vector<std::string> dirs_sort;

    // ros::Rate loop_rate(5);
    while((dirp = readdir(dp)) != NULL)
    {
        std::string s(dirp->d_name);
        if ( s.length() > 3)
            dirs.push_back(s);
    }
    closedir(dp);

    dirs_sort = sort_string(dirs);
    return dirs_sort;
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "image_publisher");
    ros::NodeHandle nh;
    image_transport::ImageTransport it(nh);
    image_transport::Publisher pub = it.advertise("camera/image", 1);

    std::string dirname;
    std::vector<std::string> dirs_sort;

    dirname = "/home/qk/Desktop/2Dlaser/5cam_calib/scene";
    dirs_sort = get_file_dir(dirname);   

    ros::Rate loop_rate(10);
    for (size_t i = 0; i < dirs_sort.size(); i++)
    {
        std::cout << dirs_sort[i] << std::endl;
        std::string img_path = dirname + "/" + dirs_sort[i];
        cv::Mat image = cv::imread(img_path, CV_LOAD_IMAGE_COLOR);
        sensor_msgs::ImagePtr msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", image).toImageMsg();
        // std::cout << " string : " << s.substr(s.length()-3, 3) << std::endl;
        pub.publish(msg);
        ros::spinOnce();
        loop_rate.sleep();
    }

}
