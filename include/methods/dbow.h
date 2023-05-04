#pragma once

#include <base.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <DBoW2/DBoW2.h>

class DBoW : public BaseMethod
{
public:
    explicit DBoW(double threshold);

    virtual void predict_loop_candidates(std::vector<std::vector<bool>> &predicted,
                                         std::vector<sensor_msgs::PointCloud2> &cloud_vec,
                                         std::vector<geometry_msgs::Pose> &odom_vec,
                                         std::vector<sensor_msgs::Image> &img_vec,
                                         int min_idx_diff) override;

    virtual const std::string getName() const override;

    static void createVocabulary(const std::vector<sensor_msgs::Image> &img_vec, std::string &voc_name);

private:
    cv::Ptr<cv::ORB> orb;
    cv_bridge::CvImagePtr cv_ptr;
    OrbVocabulary orb_vocab;
    const double THRESHOLD;
};