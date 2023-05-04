#pragma once

#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/Image.h>
#include <geometry_msgs/Pose.h>
#include <nav_msgs/Odometry.h>

#include <boost/optional.hpp>
#include <vector>
#include <string>
#include "methods/base.h"

class Evaluator
{
public:
    // Constructor with record size parameter
    Evaluator(const std::vector<BaseMethod*> &methods, int record_size,
                         bool angle_consideration, int max_angle_deg, double max_dist, int min_idx_diff, bool save_candidates);

    ~Evaluator() {}

    // Override the callback for synchronized point cloud, image, and pose messages
    void synced_callback(const sensor_msgs::PointCloud2::ConstPtr &cloud_msg,
                         const nav_msgs::Odometry::ConstPtr &odom_msg,
                         const boost::optional<sensor_msgs::Image::ConstPtr> &img_msg);

    // Get ground truth loop closure candidates
    void get_real_loop_candidates();

    // Get predicted loop closure candidates from the place recognition model
    void get_model_loop_candidates();

    // Calculate evaluation metrics: precision, recall, and F1 score
    void calculate_metrics(std::vector<std::vector<bool>> &predicted_loop_candidates, double &precision, double &recall, double &f1_score, double &accuracy);

    void plot_odometry_path(const std::vector<geometry_msgs::Pose>& odom_vec);

private:
    // Number of triplets to record
    const int RECORD_SIZE;
    const bool ANGLE_CONSIDERATION;
    const int MAX_ANGLE_DEG;
    const double MAX_ANGLE_RAD; // Maximum angle difference between loop candidates
    const double MAX_DIST;

    // Minimum index difference between loop candidates
    const int MIN_IDX_DIFF;

    // Whether to save a loop closure candidates to a file
    const bool SAVE_CANDIDATES = false;
    
    void save_candidates(std::vector<std::vector<bool>> &candidates, const std::string &filename);

    // Vectors to store recorded data
    std::vector<geometry_msgs::Pose> odom_vec;       // Pose messages
    std::vector<sensor_msgs::PointCloud2> cloud_vec; // Point cloud messages
    std::vector<sensor_msgs::Image> img_vec;         // Image messages

    // Vector to store predicted loop closure candidates
    std::vector<std::vector<bool>> real_loop_candidates;

    std::vector<BaseMethod*> methods;
};
