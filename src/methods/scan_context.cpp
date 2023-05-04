#include "methods/scan_context.h"

ScanContext::ScanContext(double threshold, double leaf_size)
    : THRESHOLD(threshold), LEAF_SIZE(leaf_size)
{
    // Set the leaf size for the voxel grid filter
    voxel_grid.setLeafSize(LEAF_SIZE, LEAF_SIZE, LEAF_SIZE);
}

const std::string ScanContext::getName() const
{
    const std::string name = "ScanContext (threshold " + std::to_string(THRESHOLD) + ", leaf size " + std::to_string(LEAF_SIZE) + ")";
    return name;
}

void ScanContext::predict_loop_candidates(std::vector<std::vector<bool>> &predicted,
                                          std::vector<sensor_msgs::PointCloud2> &cloud_vec,
                                          std::vector<geometry_msgs::Pose> &odom_vec,
                                          std::vector<sensor_msgs::Image> &img_vec,
                                          int min_idx_diff)
{
    // Create a new point cloud for storing the downsampled cloud
    pcl::PointCloud<pcl::PointXYZI>::Ptr downsampled_cloud(new pcl::PointCloud<pcl::PointXYZI>);

    for (int i = 0; i < cloud_vec.size(); ++i)
    {
        // Convert, downsample, and send all recorded clouds to the SCManager
        // Convert the ROS point cloud message to a PCL point cloud
        pcl::fromROSMsg(cloud_vec[i], *downsampled_cloud);

        // Downsample the point cloud using the voxel grid filter
        voxel_grid.setInputCloud(downsampled_cloud);
        voxel_grid.filter(*downsampled_cloud);

        // Send the downsampled point cloud to the SCManager and save the Scan Context and key
        sc_manager.makeAndSaveScancontextAndKeys(*downsampled_cloud);

        // Calculate the descriptor distances between the current and previous scans
        for (int j = 0; j <= i - min_idx_diff; ++j)
        {
            double dist = sc_manager.getDistance(i, j);
            if (dist < THRESHOLD)
                predicted[i][j] = true;
        }
    }
}