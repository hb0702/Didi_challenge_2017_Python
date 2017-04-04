#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>

void processLidarPoints(const sensor_msgs::PointCloud2::ConstPtr& msg)
{
    ROS_INFO("Got points");
//    ROS_INFO("I heard: [%s]", msg->data.c_str());

    // To do: process lidar points
    // ...
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "lidar_points_listener");
    ros::NodeHandle n;

    ros::Subscriber sub = n.subscribe("scene/lidar_points", 10, processLidarPoints);

    // To do: subscribe image processing result
    // ...

    // To do: process lidar points
    // ...

    // To do: post process (publish result?)
    // ...

    ros::spin();

    return 0;
}
