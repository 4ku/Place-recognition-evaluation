#include "methods/STD.h"
#include <pcl_conversions/pcl_conversions.h>

STD::STD(ConfigSetting &config_setting) : config_setting(config_setting)
{
}

const std::string STD::getName() const
{
    const std::string name = "STD";
    return name;
}

void STD::predict_loop_candidates(std::vector<std::vector<bool>> &predicted,
                                  std::vector<sensor_msgs::PointCloud2> &cloud_vec,
                                  std::vector<geometry_msgs::Pose> &odom_vec,
                                  std::vector<sensor_msgs::Image> &img_vec,
                                  int min_idx_diff)
{

    STDescManager *std_manager = new STDescManager(config_setting);
    for (int i = 0; i < cloud_vec.size(); ++i)
    {
        std_manager->config_setting_.candidate_num_ = i - min_idx_diff + 1;

        pcl::PointCloud<pcl::PointXYZI>::Ptr temp_cloud(new pcl::PointCloud<pcl::PointXYZI>);
        pcl::fromROSMsg(cloud_vec[i], *temp_cloud);

        std::vector<STDesc> stds_vec;
        std_manager->GenerateSTDescs(temp_cloud, stds_vec);

        std::vector<STDMatchList> candidate_matcher_vec;
        std_manager->candidate_selector(stds_vec, candidate_matcher_vec);

        for (size_t i = 0; i < candidate_matcher_vec.size(); i++)
        {
            double verify_score = -1;
            std::pair<Eigen::Vector3d, Eigen::Matrix3d> relative_pose;
            std::vector<std::pair<STDesc, STDesc>> sucess_match_vec;
            std_manager->candidate_verify(candidate_matcher_vec[i], verify_score, relative_pose,
                                          sucess_match_vec);
            if (verify_score > std_manager->config_setting_.icp_threshold_)
            {
                int candidate_id = candidate_matcher_vec[i].match_id_.second;
                predicted[i][candidate_id] = true;
            }
        }

        std_manager->AddSTDescs(stds_vec);
    }
    delete std_manager;
}


// void STD::predict_loop_candidates(std::vector<std::vector<bool>> &predicted,
//                                   std::vector<sensor_msgs::PointCloud2> &cloud_vec,
//                                   std::vector<geometry_msgs::Pose> &odom_vec,
//                                   std::vector<sensor_msgs::Image> &img_vec,
//                                   int min_idx_diff)
// {
//     static size_t keyCloudInd = 0;
//     STDescManager *std_manager = new STDescManager(config_setting);
//     for (int i = 0; i < cloud_vec.size(); ++i)
//     {
//         std_manager->config_setting_.candidate_num_ = i - min_idx_diff + 1;

//         pcl::PointCloud<pcl::PointXYZI>::Ptr temp_cloud(new pcl::PointCloud<pcl::PointXYZI>);
//         pcl::fromROSMsg(cloud_vec[i], *temp_cloud);

//         std::vector<STDesc> stds_vec;
//         std_manager->GenerateSTDescs(temp_cloud, stds_vec);



//         std::pair<int, double> search_result(-1, 0);
//         std::pair<Eigen::Vector3d, Eigen::Matrix3d> loop_transform;
//         loop_transform.first << 0, 0, 0;
//         loop_transform.second = Eigen::Matrix3d::Identity();
//         std::vector<std::pair<STDesc, STDesc>> loop_std_pair;
//         if (keyCloudInd > config_setting.skip_near_num_)
//         {
//         std_manager->SearchLoop(stds_vec, search_result, loop_transform,
//                                 loop_std_pair);
//         }
//         if (search_result.first > 0)
//         {
//         std::cout << "[Loop Detection] triggle loop: " << keyCloudInd
//                     << "--" << search_result.first
//                     << ", score:" << search_result.second << std::endl;
//         }

//         std_manager->AddSTDescs(stds_vec);
//         keyCloudInd++;
//         predicted[i][search_result.second] = true;
//     }
//     delete std_manager;
// }