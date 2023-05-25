#pragma once

#include <base.h>
#include <STD/STDesc.h>
#include <pcl/io/pcd_io.h>

class STD : public BaseMethod
{
public:
    explicit STD(ConfigSetting &config_setting);

    virtual void predict_loop_candidates(std::vector<std::vector<bool>> &predicted,
                                        std::vector<sensor_msgs::PointCloud2> &cloud_vec,
                                        std::vector<geometry_msgs::Pose> &odom_vec,
                                        std::vector<sensor_msgs::Image> &img_vec,
                                        int min_idx_diff) override;
    virtual const std::string getName() const override;

private:
    ConfigSetting config_setting;
};
