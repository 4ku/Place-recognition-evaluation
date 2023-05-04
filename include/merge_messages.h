#pragma once

#include <ros/ros.h>

#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/Image.h>
#include <nav_msgs/Odometry.h>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>

#include <boost/optional.hpp>
#include <vector>

#include <rosbag/bag.h>
#include <rosbag/view.h>

class MergeRepublisher
{
public:
    explicit MergeRepublisher(bool use_rosbag, std::string input_bag_path, std::string output_bag_path, std::string &merged_lidar_topic,
                              std::string &merged_odom_topic, std::string &merged_cam_topic, int keyframe_type, int cloud_size, double leaf_size);

    void callback(const sensor_msgs::PointCloud2::ConstPtr &cloud_msg,
                  const nav_msgs::Odometry::ConstPtr &odom_msg,
                  const boost::optional<sensor_msgs::Image::ConstPtr> &img_msg);

    pcl::PointCloud<pcl::PointXYZI> merge_cloud();

    void processBag(std::string lidar_topic, std::string odom_topic, std::string cam_topic);

private:
    std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr> cloud_vec;
    std::vector<geometry_msgs::Pose> odom_vec;
    std::vector<sensor_msgs::Image> img_vec;

    ros::NodeHandle nh_;
    ros::Publisher merged_cloud_publisher, merged_odom_publisher, merged_img_publisher;

    pcl::VoxelGrid<pcl::PointXYZI> downsampler;

    // parameters:
    const int CLOUD_SIZE;
    const double LEAF_SIZE;

    // 0 - queue like keyframes, batch like keyframes
    const int KEYFRAME_TYPE;

    const bool USE_ROSBAG;
    rosbag::Bag input_bag;
    rosbag::Bag output_bag;

    std::string merged_lidar_topic;
    std::string merged_odom_topic;
    std::string merged_cam_topic;
};
