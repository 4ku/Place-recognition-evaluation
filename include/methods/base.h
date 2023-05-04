#pragma once

#include <vector>
#include <geometry_msgs/Pose.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/Image.h>

class BaseMethod
{
public:
    virtual void predict_loop_candidates(std::vector<std::vector<bool>> &predicted,
                                         std::vector<sensor_msgs::PointCloud2> &cloud_vec,
                                         std::vector<geometry_msgs::Pose> &odom_vec,
                                         std::vector<sensor_msgs::Image> &img_vec,
                                         int min_idx_diff) = 0;

    virtual const std::string getName() const = 0;
};