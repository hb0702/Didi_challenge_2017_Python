#include "ros/ros.h"
#include "std_msgs/String.h"

void processImage(const std_msgs::String::ConstPtr& msg)
{
//    ROS_INFO("I heard: [%s]", msg->data.c_str());
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "image_listener");
    ros::NodeHandle n;

    ros::Subscriber sub = n.subscribe("scene/image", 1000, processImage);

    ros::spin();

    return 0;
}
