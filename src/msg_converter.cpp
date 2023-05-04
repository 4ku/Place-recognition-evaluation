#include <ros/ros.h>
#include <livox_ros_driver/CustomMsg.h>
#include <sensor_msgs/PointCloud2.h>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>

#include <map>
#include <string>
#include <math.h>

double calc_dist3d(pcl::PointXYZI &pt)
{
    return sqrt((pt.x * pt.x) + (pt.y * pt.y) + (pt.z * pt.z));
}

void process_cloud(const livox_ros_driver::CustomMsg::ConstPtr &msg,
                   pcl::PointCloud<pcl::PointXYZI>::Ptr &pcl_cloud_filtered,
                   bool print_info = false, double dist_threshold = 1.0)
{
    pcl::PointCloud<pcl::PointXYZI>::Ptr pcl_cloud(new pcl::PointCloud<pcl::PointXYZI>);

    int high_noise = 0;

    std::map<std::string, int> point_info;
    point_info["ret_0"] = 0;
    point_info["ret_1"] = 0;
    point_info["ret_2"] = 0;
    point_info["ret_3"] = 0;
    point_info["noise"] = 0;

    for (size_t i = 0; i < msg->point_num; i++)
    {
        // 2 types of points are considered good:
        // 1. normal point(spatial position), normal point(intensity), return
        if ((msg->points[i].tag & 0x30) == 0x30)
        {
            point_info["ret_3"]++;
        }
        else if ((msg->points[i].tag & 0x30) == 0x20)
        {
            point_info["ret_2"]++;
        }
        else if ((msg->points[i].tag & 0x30) == 0x10)
        {
            point_info["ret_1"]++;

            pcl::PointXYZI cur_point;
            cur_point.x = msg->points[i].x;
            cur_point.y = msg->points[i].y;
            cur_point.z = msg->points[i].z;
            cur_point.intensity = msg->points[i].reflectivity;
            pcl_cloud->push_back(cur_point);
        }
        else if ((msg->points[i].tag & 0x30) == 0x00)
        {
            // ret_num0++;
            point_info["ret_0"]++;

            pcl::PointXYZI cur_point;
            cur_point.x = msg->points[i].x;
            cur_point.y = msg->points[i].y;
            cur_point.z = msg->points[i].z;
            cur_point.intensity = msg->points[i].reflectivity;
            pcl_cloud->push_back(cur_point);
        }

        if (msg->points[i].tag % 16 != 0)
        {
            // high_noise++;
            point_info["noise"]++;
        }
    }

    // pcl::PointCloud<pcl::PointXYZI>::Ptr pcl_cloud_filtered(new pcl::PointCloud<pcl::PointXYZI>);
    // 2 filters:
    //      1. remove points that are too close to lidar
    //      2. remove one of the points if 2 points are too close to each other
    for (size_t i = 0; i < pcl_cloud->size(); i++)
    {
        if (calc_dist3d(pcl_cloud->points[i]) < dist_threshold)
        {
            continue;
        }

        pcl_cloud_filtered->push_back(pcl_cloud->points[i]);
    }

    int normal_points = point_info["ret_0"] + point_info["ret_1"] + point_info["ret_2"] + point_info["ret_3"];

    if (print_info)
    {
        std::cout << "ret_num0: " << point_info["ret_0"] << "| ret_num1: " << point_info["ret_1"] << "| ret_num2: "
                  << point_info["ret_2"] << "| ret_num3: " << point_info["ret_3"] << std::endl;
        std::cout << "normal: " << normal_points << "| noise: " << high_noise << std::endl;
    }
}

class msgConverter
{

public:
    ros::NodeHandle nh;
    ros::Publisher lidar_pub;
    ros::Subscriber lidar_sub;

    ros::Time last_time;

    msgConverter()
    {
        lidar_pub = nh.advertise<sensor_msgs::PointCloud2>("/livox/lidar_pc", 10000);
        lidar_sub = nh.subscribe<livox_ros_driver::CustomMsg>("/livox/lidar", 10000, &msgConverter::convert_cloud, this);
    }

    void convert_cloud(const livox_ros_driver::CustomMsg::ConstPtr &input_cloud)
    {
        // custom->pointcloud2

        pcl::PointCloud<pcl::PointXYZI>::Ptr pcl_cloud(new pcl::PointCloud<pcl::PointXYZI>);
        process_cloud(input_cloud, pcl_cloud);

        sensor_msgs::PointCloud2 temp_out_msg;

        pcl::toROSMsg(*pcl_cloud, temp_out_msg);
        last_time = ros::Time::now();

        temp_out_msg.header.stamp = input_cloud->header.stamp;

        temp_out_msg.header.frame_id = input_cloud->header.frame_id;
        temp_out_msg.header.frame_id = "trunk";

        lidar_pub.publish(temp_out_msg);
    }
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "cloud_converter");

    msgConverter converter;

    ros::spin();

    return 0;
}