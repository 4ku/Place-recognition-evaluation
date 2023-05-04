#include "methods/dbow.h"
#include <image_transport/image_transport.h>

void imageCallback(const sensor_msgs::ImageConstPtr &msg)
{
    static std::vector<sensor_msgs::Image> images;
    static int counter = 0;

    try
    {
        counter++;
        if (counter % 5 != 0)
            return;
        images.push_back(*msg);

        if (images.size() % 10 == 0)
            ROS_INFO("Received images: %lu", images.size());

        if (images.size() >= 500)
        {
            std::string voc_name = "orb_vocab.yml.gz";
            DBoW::createVocabulary(images, voc_name);

            // Shutdown the node after the vocabulary is created and saved
            ros::shutdown();
        }
    }
    catch (cv_bridge::Exception &e)
    {
        ROS_ERROR("Could not convert from '%s' to 'bgr8'.", msg->encoding.c_str());
    }
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "image_listener");
    ros::NodeHandle nh;

    image_transport::ImageTransport it(nh);
    image_transport::Subscriber sub = it.subscribe("/camera/color/image_raw_sync", 1, imageCallback);

    ros::spin();

    return 0;
}