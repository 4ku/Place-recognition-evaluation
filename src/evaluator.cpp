#include "evaluator.h"

#include <ros/ros.h>
#include "ros/package.h"

#include <fstream>
#include <cmath>
#include <chrono>
#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>
#include <iostream>
#include <boost/filesystem.hpp>

/**
 * \brief Calculate the angular difference between two quaternions
 *
 * \param quat_1 The first quaternion
 * \param quat_2 The second quaternion
 *
 * \return The angular difference between quat_1 and quat_2
 */
static double angular_difference(const geometry_msgs::Quaternion &quat_1, const geometry_msgs::Quaternion &quat_2)
{
    const double cos_angle = quat_1.x * quat_2.x + quat_1.y * quat_2.y + quat_1.z * quat_2.z + quat_1.w * quat_2.w;
    const double angle = 2 * acos(abs(cos_angle)); // use absolute value to avoid NaNs due to rounding errors
    return angle;
}

/**
 * \brief Calculate the Euclidean distance between two points
 *
 * \param point_1 The first point
 * \param point_2 The second point
 *
 * \return The Euclidean distance between point_1 and point_2
 */
static double euclidean_difference(const geometry_msgs::Point &point_1, const geometry_msgs::Point &point_2)
{
    const double dx = point_1.x - point_2.x;
    const double dy = point_1.y - point_2.y;
    const double dz = point_1.z - point_2.z;
    const double dist = sqrt(dx * dx + dy * dy + dz * dz);
    return dist;
}

Evaluator::Evaluator(const std::vector<BaseMethod *> &methods, int record_size, bool angle_consideration, int max_angle_deg, double max_dist, int min_idx_diff, bool save_candidates)
    : methods(methods), RECORD_SIZE(record_size), ANGLE_CONSIDERATION(angle_consideration),
      MAX_ANGLE_DEG(max_angle_deg), MAX_DIST(max_dist), MIN_IDX_DIFF(min_idx_diff), MAX_ANGLE_RAD(((double)max_angle_deg / 180) * 3.14), SAVE_CANDIDATES(save_candidates)
{
    real_loop_candidates.resize(RECORD_SIZE, std::vector<bool>(RECORD_SIZE, false));
}

void Evaluator::synced_callback(const sensor_msgs::PointCloud2::ConstPtr &cloud_msg,
                                const nav_msgs::Odometry::ConstPtr &odom_msg,
                                const boost::optional<sensor_msgs::Image::ConstPtr> &img_msg)
{
    static int counter = 0;
    static int log_frequency = 50;

    if (counter % log_frequency == 0)
    {
        ROS_INFO("Amount of data recorded: %d", counter);
    }

    // Store data to lists
    cloud_vec.push_back(*cloud_msg);
    odom_vec.push_back(odom_msg->pose.pose);
    if (img_msg)
    {
        img_vec.push_back(**img_msg);
    }

    counter++;

    if (counter == RECORD_SIZE)
    {
        get_real_loop_candidates();
        get_model_loop_candidates();

        plot_odometry_path(odom_vec);
        ros::shutdown();
    }
}

static std::string getFirstWord(const std::string &input)
{
    // Find the first space character
    size_t firstSpace = input.find_first_of(' ');

    // If there is no space character, the entire string is considered one word
    if (firstSpace == std::string::npos)
    {
        return input;
    }

    // Get the substring from the start of the string to the first space character
    return input.substr(0, firstSpace);
}

void Evaluator::save_candidates(std::vector<std::vector<bool>> &candidates, const std::string &filename){
    if (SAVE_CANDIDATES)
    {
        std::ofstream file;
        std::string path = ros::package::getPath("place_recog_eval") + "/results/";

        if (ANGLE_CONSIDERATION) {
            path += "with_angle/";
        } else {
            path += "no_angle/";
        }

        // Check if the directory exists and create it if it doesn't
        if (!boost::filesystem::exists(path)) {
            boost::filesystem::create_directories(path);
        }
        path += filename;

        file.open(path);

        for (int i = 1; i < RECORD_SIZE; i++)
        {
            for (int j = 0; j <= i - MIN_IDX_DIFF; j++)
            {
                file << candidates[i][j] << " ";
            }
            file << "\n";
        }
    }
}


void Evaluator::get_real_loop_candidates()
{
    // Calculate real loop candidates
    for (int i = 1; i < RECORD_SIZE; i++)
    {
        for (int j = 0; j <= i - MIN_IDX_DIFF; j++)
        {
            double odom_diff = euclidean_difference(odom_vec[i].position, odom_vec[j].position);
            bool condition = odom_diff < MAX_DIST;

            if (ANGLE_CONSIDERATION)
            {
                double ang_diff = angular_difference(odom_vec[i].orientation, odom_vec[j].orientation);
                condition = condition && (ang_diff < MAX_ANGLE_RAD);
            }

            if (condition)
                real_loop_candidates[i][j] = true;
        }
    }

    // Save real loop candidates to file
    std::string method_name = getFirstWord(methods[0]->getName());
    save_candidates(real_loop_candidates, "real_" + method_name + ".txt");
}

void Evaluator::get_model_loop_candidates()
{   
    double best_f1_score = 0;
    BaseMethod *best_method = nullptr;

    for (BaseMethod *method : methods)
    {
        // Predict loop candidates
        std::vector<std::vector<bool>> predicted_loop_candidates(RECORD_SIZE, std::vector<bool>(RECORD_SIZE, false));

        auto start = std::chrono::high_resolution_clock::now();
        method->predict_loop_candidates(predicted_loop_candidates, cloud_vec, odom_vec, img_vec, MIN_IDX_DIFF);
        auto duration = std::chrono::high_resolution_clock::now() - start;

        ROS_INFO("Results for method: %s", method->getName().c_str());
        ROS_INFO("It took %f seconds to predict loop candidates (%lu frames total).",
                 std::chrono::duration<double>(duration).count(), cloud_vec.size());

        ROS_INFO("It took %f seconds to predict one loop candidate.",
                 std::chrono::duration<double>(duration).count() / cloud_vec.size());

        // Calculate metrics
        double precision, recall, f1_score, accuracy;
        calculate_metrics(predicted_loop_candidates, precision, recall, f1_score, accuracy);

        // Display results
        ROS_INFO("----------------------------------------------");
        ROS_INFO("Precision: %.3f", precision);
        ROS_INFO("Recall   : %.3f", recall);
        ROS_INFO("F1 Score : %.3f", f1_score);
        ROS_INFO("Accuracy : %.3f", accuracy);
        ROS_INFO("----------------------------------------------");
        ROS_INFO("----------------------------------------------");
        ROS_INFO(" ");

        if (f1_score > best_f1_score)
        {
            best_f1_score = f1_score;
            best_method = method;
        }

        // Save predicted loop candidates to file
        save_candidates(predicted_loop_candidates, method->getName() + ".txt");
    }

    ROS_INFO("Best method: %s with f1 score: %.3f", best_method->getName().c_str(), best_f1_score);
}

void Evaluator::calculate_metrics(std::vector<std::vector<bool>> &predicted_loop_candidates, double &precision, double &recall, double &f1_score, double &accuracy)
{
    int tp = 0, fp = 0, fn = 0, tn = 0;

    for (int i = 1; i < RECORD_SIZE; i++)
    {
        for (int j = 0; j <= i - MIN_IDX_DIFF; j++)
        {
            bool predicted = predicted_loop_candidates[i][j];
            bool real = real_loop_candidates[i][j];

            if (predicted && real)
                tp++;
            else if (predicted && !real)
                fp++;
            else if (!predicted && real)
                fn++;
            else if (!predicted && !real)
                tn++;
        }
    }
    precision = (tp + fp) > 0 ? static_cast<double>(tp) / (tp + fp) : 0;
    recall = (tp + fn) > 0 ? static_cast<double>(tp) / (tp + fn) : 0;
    f1_score = (precision + recall) > 0 ? 2 * precision * recall / (precision + recall) : 0;
    accuracy = (tp + tn + fp + fn) > 0 ? static_cast<double>(tp + tn) / (tp + tn + fp + fn) : 0;
}

void Evaluator::plot_odometry_path(const std::vector<geometry_msgs::Pose> &odom_vec)
{
    // Create a blank image
    int img_size = 512;
    cv::Mat img(img_size, img_size, CV_8UC3, cv::Scalar(255, 255, 255));

    // Find min and max coordinates of the path
    double min_x = std::numeric_limits<double>::max();
    double min_y = std::numeric_limits<double>::max();
    double max_x = std::numeric_limits<double>::min();
    double max_y = std::numeric_limits<double>::min();

    for (const auto &pose : odom_vec)
    {
        min_x = std::min(min_x, pose.position.x);
        min_y = std::min(min_y, pose.position.y);
        max_x = std::max(max_x, pose.position.x);
        max_y = std::max(max_y, pose.position.y);
    }

    // Calculate scale and offsets
    double scale = std::min(img_size / (max_x - min_x), img_size / (max_y - min_y));
    double x_offset = (img_size - (max_x - min_x) * scale) / 2.0 - min_x * scale;
    double y_offset = (img_size - (max_y - min_y) * scale) / 2.0 - min_y * scale;

    // Plot each point
    cv::Point2i prev_point;
    bool prev_point_valid = false;
    for (const auto &pose : odom_vec)
    {
        cv::Point2i point(
            static_cast<int>(pose.position.x * scale + x_offset),
            static_cast<int>(img_size - (pose.position.y * scale + y_offset)));

        cv::circle(img, point, 2, cv::Scalar(0, 0, 255), -1);

        // Draw a line between the previous point and the current point, if the previous point is valid
        if (prev_point_valid)
        {
            cv::line(img, prev_point, point, cv::Scalar(255, 0, 0), 1);
        }

        // Update the previous point and set it as valid
        prev_point = point;
        prev_point_valid = true;
    }

    // Save the image to a file
    std::string path = ros::package::getPath("place_recog_eval");
    std::string filename = "odometry_path.png";
    cv::imwrite(path + "/" + filename, img);
}