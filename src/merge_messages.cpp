
#include "merge_messages.h"
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

#include <pcl_conversions/pcl_conversions.h>
#include <pcl/common/transforms.h>

#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <tf/tf.h>

#include <map>

static Eigen::Matrix<double, 3, 3> quat2rot(const geometry_msgs::Quaternion &quat)
{
    tf::Quaternion q(
        quat.x,
        quat.y,
        quat.z,
        quat.w);

    tf::Matrix3x3 m(q);

    Eigen::Matrix<double, 3, 3> rot;

    rot << m[0][0], m[0][1], m[0][2],
        m[1][0], m[1][1], m[1][2],
        m[2][0], m[2][1], m[2][2];

    return rot;
}

static Eigen::Matrix<double, 4, 4> quatpos2mat(const geometry_msgs::Quaternion &quat, const geometry_msgs::Point &pos)
{
    Eigen::Matrix<double, 3, 3> rot = quat2rot(quat);

    Eigen::Matrix<double, 4, 4> mat = Eigen::Matrix4d::Identity();

    for (size_t i = 0; i < rot.rows(); i++)
    {
        for (size_t j = 0; j < rot.cols(); j++)
        {
            mat(i, j) = rot(i, j);
        }
    }
    mat(0, 3) = pos.x;
    mat(1, 3) = pos.y;
    mat(2, 3) = pos.z;

    return mat;
}

MergeRepublisher::MergeRepublisher(bool use_rosbag, std::string input_bag_path, std::string output_bag_path,
                                   std::string &merged_lidar_topic, std::string &merged_odom_topic, std::string &merged_cam_topic,
                                   int keyframe_type, int cloud_size, double leaf_size) : USE_ROSBAG(use_rosbag), merged_lidar_topic(merged_lidar_topic),
                                                                                          merged_odom_topic(merged_odom_topic), merged_cam_topic(merged_cam_topic),
                                                                                          KEYFRAME_TYPE(keyframe_type), CLOUD_SIZE(cloud_size), LEAF_SIZE(leaf_size)
{
    if (USE_ROSBAG)
    {
        try
        {
            input_bag.open(input_bag_path, rosbag::bagmode::Read);
            output_bag.open(output_bag_path, rosbag::bagmode::Write);
        }
        catch (rosbag::BagException &e)
        {
            ROS_ERROR("Error opening bag file: %s", e.what());
        }
    }
    else
    {
        merged_cloud_publisher = nh_.advertise<sensor_msgs::PointCloud2>(merged_lidar_topic, 20);
        merged_odom_publisher = nh_.advertise<nav_msgs::Odometry>(merged_odom_topic, 20);
        merged_img_publisher = nh_.advertise<sensor_msgs::Image>(merged_cam_topic, 20);
    }

    downsampler.setLeafSize(LEAF_SIZE, LEAF_SIZE, LEAF_SIZE);
}

void MergeRepublisher::callback(const sensor_msgs::PointCloud2::ConstPtr &cloud_msg,
                                const nav_msgs::Odometry::ConstPtr &odom_msg,
                                const boost::optional<sensor_msgs::Image::ConstPtr> &img_msg)
{
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZI>());
    pcl::fromROSMsg(*cloud_msg, *cloud);

    cloud_vec.push_back(cloud);
    odom_vec.push_back(odom_msg->pose.pose);

    if (img_msg)
        img_vec.push_back(**img_msg);

    if (cloud_vec.size() >= CLOUD_SIZE)
    {
        // Publish data
        sensor_msgs::PointCloud2 cloud_msg;
        nav_msgs::Odometry odom_msg;
        sensor_msgs::Image img_msg;

        pcl::PointCloud<pcl::PointXYZI> cloud_total = merge_cloud();

        pcl::toROSMsg(cloud_total, cloud_msg);
        cloud_msg.header.frame_id = "trunk";

        odom_msg.pose.pose = odom_vec[0];
        odom_msg.header.stamp = cloud_msg.header.stamp;
        odom_msg.header.frame_id = cloud_msg.header.frame_id;

        if (!img_vec.empty())
        {
            img_msg = img_vec[0];
            img_msg.header.stamp = cloud_msg.header.stamp;
            img_msg.header.frame_id = cloud_msg.header.frame_id;
        }

        if (USE_ROSBAG)
        {
            output_bag.write(merged_lidar_topic, cloud_msg.header.stamp, cloud_msg);
            output_bag.write(merged_odom_topic, cloud_msg.header.stamp, odom_msg);
            output_bag.write(merged_cam_topic, cloud_msg.header.stamp, img_msg);
        }
        else
        {
            merged_cloud_publisher.publish(cloud_msg);
            merged_odom_publisher.publish(odom_msg);
            if (!img_vec.empty())
                merged_img_publisher.publish(img_msg);
        }

        // Update buffers
        size_t erase_count = (KEYFRAME_TYPE == 0) ? 1 : CLOUD_SIZE;
        cloud_vec.erase(cloud_vec.begin(), cloud_vec.begin() + erase_count);
        odom_vec.erase(odom_vec.begin(), odom_vec.begin() + erase_count);
        if (!img_vec.empty())
            img_vec.erase(img_vec.begin(), img_vec.begin() + erase_count);
    }
}

// Merges point clouds in the cloud_vec and applies a voxel grid downsampling filter.
// Transforms all clouds into local frame
pcl::PointCloud<pcl::PointXYZI> MergeRepublisher::merge_cloud()
{
    pcl::PointCloud<pcl::PointXYZI> cloud_total;

    // Get the SE3 matrix for the first cloud
    Eigen::Matrix<double, 4, 4> tf_mat1 = quatpos2mat(odom_vec[0].orientation, odom_vec[0].position);

    // Add the first cloud to the resulting cloud
    cloud_total += *cloud_vec[0];

    // Iterate through the remaining clouds
    for (int i = 1; i < cloud_vec.size(); i++)
    {
        // Get the SE3 matrix for the current cloud
        Eigen::Matrix<double, 4, 4> tf_mat2 = quatpos2mat(odom_vec[i].orientation, odom_vec[i].position);

        // Compute the relative transformation between the first cloud and the current cloud
        Eigen::Matrix<double, 4, 4> tf_mat12 = tf_mat1.inverse() * tf_mat2;

        // Create a transformation matrix from the relative transformation
        Eigen::Matrix4d transform = Eigen::Matrix4d::Identity();
        transform.block(0, 0, 3, 3) = tf_mat12.block(0, 0, 3, 3);
        transform.block(0, 3, 3, 1) = tf_mat12.block(0, 3, 3, 1);

        // Transform the current cloud into the first cloud's frame
        pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_transformed(new pcl::PointCloud<pcl::PointXYZI>());
        pcl::transformPointCloud(*cloud_vec[i], *cloud_transformed, transform);

        // Add the transformed cloud to the resulting cloud
        cloud_total += *cloud_transformed;
    }

    // Apply the voxel grid downsampling filter to the merged cloud
    downsampler.setInputCloud(cloud_total.makeShared());
    downsampler.filter(cloud_total);

    return cloud_total;
}

void MergeRepublisher::processBag(std::string lidar_topic, std::string odom_topic, std::string cam_topic)
{
    // Specify topics to read from the rosbag
    std::vector<std::string> topics = {lidar_topic, odom_topic, cam_topic};
    rosbag::View view(input_bag, rosbag::TopicQuery(topics));

    sensor_msgs::PointCloud2::ConstPtr cloud_msg;
    nav_msgs::Odometry::ConstPtr odom_msg;
    sensor_msgs::Image::ConstPtr img_msg;

    std::multimap<ros::Time, nav_msgs::Odometry::ConstPtr> odom_map;
    std::multimap<ros::Time, sensor_msgs::Image::ConstPtr> img_map;

    ros::Duration max_time_diff(0.1); // Adjust this value as needed
    int count = 0;

    for (const rosbag::MessageInstance &m : view)
    {
        if (m.getTopic() == lidar_topic)
        {
            cloud_msg = m.instantiate<sensor_msgs::PointCloud2>();

            // Find an odom_msg and img_msg that are close in time to the cloud_msg
            nav_msgs::Odometry::ConstPtr odom_msg;
            sensor_msgs::Image::ConstPtr img_msg;

            auto odom_iter = odom_map.lower_bound(cloud_msg->header.stamp - max_time_diff);
            if (odom_iter != odom_map.end() && (odom_iter->first - cloud_msg->header.stamp) <= max_time_diff)
            {
                odom_msg = odom_iter->second;
                odom_map.erase(odom_iter);
            }

            auto img_iter = img_map.lower_bound(cloud_msg->header.stamp - max_time_diff);
            if (img_iter != img_map.end() && (img_iter->first - cloud_msg->header.stamp) <= max_time_diff)
            {
                img_msg = img_iter->second;
                img_map.erase(img_iter);
            }

            if (odom_msg && img_msg)
            {
                count++;
                callback(cloud_msg, odom_msg, img_msg);
            }
        }
        else if (m.getTopic() == odom_topic)
        {
            odom_msg = m.instantiate<nav_msgs::Odometry>();
            odom_map.insert({odom_msg->header.stamp, odom_msg});
        }
        else if (m.getTopic() == cam_topic)
        {
            img_msg = m.instantiate<sensor_msgs::Image>();
            img_map.insert({img_msg->header.stamp, img_msg});
        }
    }

    ROS_INFO("Processed %d messages", count);

    input_bag.close();
    output_bag.close();
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "merge_republisher");

    ros::NodeHandle nh;

    std::string lidar_topic, odom_topic, cam_topic;
    nh.getParam("lidar_raw_topic", lidar_topic);
    nh.getParam("odom_raw_topic", odom_topic);
    nh.getParam("img_raw_topic", cam_topic);

    std::string merged_lidar_topic, merged_odom_topic, merged_cam_topic;
    nh.getParam("merged_lidar_topic", merged_lidar_topic);
    nh.getParam("merged_odom_topic", merged_odom_topic);
    nh.getParam("merged_camera_topic", merged_cam_topic);

    int keyframe_type;
    nh.getParam("keyframe_type", keyframe_type);

    int cloud_size;
    nh.getParam("cloud_size", cloud_size);

    double leaf_size;
    nh.getParam("merge_leaf_size", leaf_size);

    bool use_rosbag;
    nh.getParam("use_rosbag", use_rosbag);

    std::string rosbag_input_path;
    nh.getParam("merge_rosbag_input_path", rosbag_input_path);

    std::string rosbag_output_path;
    nh.getParam("merge_rosbag_output_path", rosbag_output_path);

    MergeRepublisher merge_republisher(use_rosbag, rosbag_input_path, rosbag_output_path,
                                       merged_lidar_topic, merged_odom_topic, merged_cam_topic,
                                       keyframe_type, cloud_size, leaf_size);

    if (use_rosbag)
    {
        merge_republisher.processBag(lidar_topic, odom_topic, cam_topic);
        ROS_INFO("Rosbag saved to %s", rosbag_output_path.c_str());
    }
    else
    {
        message_filters::Subscriber<sensor_msgs::PointCloud2> cloud_sub(nh, lidar_topic, 1000);
        message_filters::Subscriber<nav_msgs::Odometry> odom_sub(nh, odom_topic, 2000);
        message_filters::Subscriber<sensor_msgs::Image> img_sub(nh, cam_topic, 1000);

        typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::PointCloud2, nav_msgs::Odometry, sensor_msgs::Image> MySyncPolicy;
        message_filters::Synchronizer<MySyncPolicy> sync(MySyncPolicy(100), cloud_sub, odom_sub, img_sub);
        sync.registerCallback(boost::bind(&MergeRepublisher::callback, &merge_republisher, _1, _2, _3));

        ros::spin();
    }

    return 0;
}