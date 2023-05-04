#pragma once

#include <base.h>
#include <scan_context/Scancontext.h>
#include <pcl/io/pcd_io.h>

class ScanContext : public BaseMethod
{
public:
    explicit ScanContext(double threshold, double leaf_size);

    virtual void predict_loop_candidates(std::vector<std::vector<bool>> &predicted,
                                        std::vector<sensor_msgs::PointCloud2> &cloud_vec,
                                        std::vector<geometry_msgs::Pose> &odom_vec,
                                        std::vector<sensor_msgs::Image> &img_vec,
                                        int min_idx_diff) override;
    
    virtual const std::string getName() const override;

private:
    pcl::VoxelGrid<pcl::PointXYZI> voxel_grid;

    SCManager sc_manager;

    const double LEAF_SIZE;
    const double THRESHOLD;
};
