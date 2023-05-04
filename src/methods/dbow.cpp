#include "methods/dbow.h"
#include <ros/ros.h>
#include "ros/package.h"

DBoW::DBoW(double threshold) : THRESHOLD(threshold)
{
    orb = cv::ORB::create();

    std::string path = ros::package::getPath("place_recog_eval");
    std::string voc_path = path + "/include/methods/dbow/orb_vocab.yml.gz";

    orb_vocab.load(voc_path);
}

const std::string DBoW::getName() const
{
    const std::string name = "DBoW (threshold " + std::to_string(THRESHOLD) + ")";
    return name;
}

void DBoW::predict_loop_candidates(std::vector<std::vector<bool>> &predicted,
                                   std::vector<sensor_msgs::PointCloud2> &cloud_vec,
                                   std::vector<geometry_msgs::Pose> &odom_vec,
                                   std::vector<sensor_msgs::Image> &img_vec,
                                   int min_idx_diff)
{
    std::vector<DBoW2::BowVector> desc_vec;

    // Create a DBoW2 scorer for L1 distance
    DBoW2::L1Scoring scorer;

    // Extract features from all prerecorded images
    for (int i = 0; i < img_vec.size(); ++i)
    {
        // Convert the image message to a CvImagePtr
        cv_ptr = cv_bridge::toCvCopy(img_vec[i], sensor_msgs::image_encodings::MONO8);

        // Detect keypoints and compute ORB descriptors
        std::vector<cv::KeyPoint> keypoints;
        cv::Mat descriptors;
        cv::Mat mask;
        orb->detectAndCompute(cv_ptr->image, mask, keypoints, descriptors);

        // Convert descriptors to a vector of Mats
        std::vector<cv::Mat> features_curr;
        for (size_t i = 0; i < descriptors.rows; i++)
        {
            features_curr.push_back(descriptors.row(i));
        }

        // Transform the features using the ORB vocabulary
        DBoW2::BowVector vec_curr;
        orb_vocab.transform(features_curr, vec_curr);

        // Add the transformed feature vector to the descriptor vector
        desc_vec.push_back(vec_curr);

        for (int j = 0; j <= i - min_idx_diff; ++j)
        {
            double dist = 1 / (scorer.score(desc_vec[i], desc_vec[j]));
            if (dist < THRESHOLD)
                predicted[i][j] = true;
        }
    }
}

void DBoW::createVocabulary(const std::vector<sensor_msgs::Image> &img_vec, std::string &voc_name)
{
    cv::Ptr<cv::ORB> orb = cv::ORB::create();

    std::string path = ros::package::getPath("place_recog_eval");
    std::string voc_path = path + "/include/methods/dbow/" + voc_name;

    // Extract features from all images in img_vec and store them in a vector<vector<cv::Mat>>
    std::vector<std::vector<cv::Mat>> features;
    features.reserve(img_vec.size());

    for (const auto &img_msg : img_vec)
    {
        // Convert the image message to a CvImagePtr
        cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(img_msg, sensor_msgs::image_encodings::MONO8);

        // Detect keypoints and compute ORB descriptors
        std::vector<cv::KeyPoint> keypoints;
        cv::Mat descriptors;
        cv::Mat mask;
        orb->detectAndCompute(cv_ptr->image, mask, keypoints, descriptors);

        // Convert descriptors to a vector of Mats
        std::vector<cv::Mat> features_curr;
        for (size_t i = 0; i < descriptors.rows; i++)
        {
            features_curr.push_back(descriptors.row(i));
        }

        // Add the features to the main features vector
        features.push_back(features_curr);
    }

    // Create the vocabulary
    const int k = 9;
    const int L = 3;
    const DBoW2::WeightingType weight = DBoW2::WeightingType::TF_IDF;
    const DBoW2::ScoringType scoring = DBoW2::ScoringType::L1_NORM;
    OrbVocabulary voc = OrbVocabulary(k, L, weight, scoring);
    voc.create(features);

    // Save the vocabulary to a file
    voc.save(voc_path);
}

