#include "methods/scan_context.h"
#include "methods/dbow.h"
#include "evaluator.h"

#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/time_synchronizer.h>

#include <rosbag/bag.h>
#include <rosbag/view.h>

void getMethods(std::vector<BaseMethod *> &methods, ros::NodeHandle &nh)
{
    std::string method_name;

    if (nh.getParam("method", method_name))
    {
        double threshold, leaf_size;
        nh.getParam("threshold", threshold);
        if (method_name == "dbow")
            methods.push_back(new DBoW(threshold));
        else if (method_name == "context")
        {
            nh.getParam("leaf_size", leaf_size);
            methods.push_back(new ScanContext(threshold, leaf_size));
        }
        else
        {
            ROS_ERROR("Invalid method parameter. Use 'dbow' or 'context'.");
            exit(-1);
        }
    }
    else
    {
        // BaseMethod *method1 = new DBoW(2.2);
        // methods.push_back(method1);

        // BaseMethod *method2 = new ScanContext(0.06, 0.2);
        // methods.push_back(method2);

        // Find best threshold for Scan Context
        for (double threshold = 0.01; threshold < 0.1; threshold += 0.01)
        {
            BaseMethod *method = new ScanContext(threshold, 0.2);
            methods.push_back(method);
        }

        // Find best threshold for DBoW
        // for (double threshold = 1.8; threshold <= 2.5; threshold += 0.1)
        // {
        //     BaseMethod *method = new DBoW(threshold);
        //     methods.push_back(method);
        // }
    }
}

void processBag(Evaluator &evaluator, std::string input_bag_path, std::string lidar_topic, std::string odom_topic, std::string camera_topic)
{
    rosbag::Bag bag;
    try
    {
        bag.open(input_bag_path, rosbag::bagmode::Read);
    }
    catch (rosbag::BagException &e)
    {
        ROS_ERROR("Error opening bag file: %s", e.what());
    }

    // Specify topics to read from the rosbag
    std::vector<std::string> topics = {lidar_topic, odom_topic, camera_topic};
    rosbag::View view(bag, rosbag::TopicQuery(topics));

    sensor_msgs::PointCloud2::ConstPtr cloud_msg;
    nav_msgs::Odometry::ConstPtr odom_msg;
    sensor_msgs::Image::ConstPtr img_msg;

    for (const rosbag::MessageInstance &m : view)
    {
        if (m.getTopic() == lidar_topic)
        {
            cloud_msg = m.instantiate<sensor_msgs::PointCloud2>();
        }
        else if (m.getTopic() == odom_topic)
        {
            odom_msg = m.instantiate<nav_msgs::Odometry>();
        }
        else if (m.getTopic() == camera_topic)
        {
            img_msg = m.instantiate<sensor_msgs::Image>();
        }

        if (cloud_msg && odom_msg && img_msg)
        {
            evaluator.synced_callback(cloud_msg, odom_msg, img_msg);
            cloud_msg.reset();
            odom_msg.reset();
            img_msg.reset();
        }
    }

    bag.close();
}

int main(int argc, char **argv)
{
    // Initialize ROS node
    ros::init(argc, argv, "evaluation_node");
    ros::NodeHandle nh;

    // Retrieve parameters from the ROS parameter server
    std::string method_name;
    int record_size, keyframe_type, cloud_size, min_idx_diff, max_angle_deg;
    double max_dist, threshold;
    bool angle_consideration, save_candidates;

    // Merge params
    nh.getParam("keyframe_type", keyframe_type);
    nh.getParam("cloud_size", cloud_size);

    // Common params
    nh.getParam("record_size", record_size);
    nh.getParam("min_idx_diff", min_idx_diff);
    nh.getParam("angle_consideration", angle_consideration);
    nh.getParam("max_angle_deg", max_angle_deg);
    nh.getParam("max_dist", max_dist);
    nh.getParam("save_candidates", save_candidates);

    if (keyframe_type == 1)
        record_size /= cloud_size;

    // Get the method parameters
    std::vector<BaseMethod *> methods;
    getMethods(methods, nh);

    // Create the evaluation object based on the selected methods
    Evaluator evaluator(methods, record_size, angle_consideration, max_angle_deg, max_dist, min_idx_diff, save_candidates);

    // Get merged topic names
    std::string lidar_topic, odom_topic, camera_topic;
    nh.getParam("merged_lidar_topic", lidar_topic);
    nh.getParam("merged_odom_topic", odom_topic);
    nh.getParam("merged_camera_topic", camera_topic);

    // Get rosbag params
    bool use_rosbag;
    std::string input_bag_path;
    nh.getParam("use_rosbag", use_rosbag);
    nh.getParam("merge_rosbag_output_path", input_bag_path);

    if (use_rosbag)
    {
        processBag(evaluator, input_bag_path, lidar_topic, odom_topic, camera_topic);
    }
    else
    {
        message_filters::Subscriber<sensor_msgs::PointCloud2> cloud_sub(nh, lidar_topic, 20);
        message_filters::Subscriber<nav_msgs::Odometry> odom_sub(nh, odom_topic, 100);
        message_filters::Subscriber<sensor_msgs::Image> img_sub(nh, camera_topic, 20);

        // Synchronize messages from different topics
        message_filters::TimeSynchronizer<sensor_msgs::PointCloud2, nav_msgs::Odometry, sensor_msgs::Image> sync(cloud_sub, odom_sub, img_sub, 100);

        // Register the synchronized callback
        sync.registerCallback(boost::bind(&Evaluator::synced_callback, &evaluator, _1, _2, _3));

        // Process callbacks and wait for new messages
        ros::spin();
    }

    return 0;
}
