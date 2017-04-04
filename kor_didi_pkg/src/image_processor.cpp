#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

void processImage(const sensor_msgs::Image::ConstPtr& msg)
{
    ROS_INFO("Got image");

    cv_bridge::CvImageConstPtr cvPtr;
    cv::Mat rgbMat;
    try
    {
        cvPtr = cv_bridge::toCvShare(msg, sensor_msgs::image_encodings::BAYER_GRBG8);
        cv::cvtColor(cvPtr->image, rgbMat, CV_BayerGR2RGB);
        ROS_INFO("Converted image");
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }

    // test: show image
//    cv::imshow("bgr", cvPtr->image);
//    cv::waitKey(10);

    // test: save image
//    std::string imagePath("/home/parkjaeil0108/challenge/Didi-Training-Release-1/test_image.jpg");
//    cv::imwrite(imagePath, rgbMat);
//    ROS_INFO("Saved image");

    // To do: process image
    // ...

    // To do: publish result
    // ...
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "image_listener");
    ros::NodeHandle n;

    ros::Subscriber sub = n.subscribe("scene/image", 10, processImage);

    ros::spin();

    return 0;
}
