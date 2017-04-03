#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>

void processLidarPoints(const sensor_msgs::PointCloud2::ConstPtr& msg)
{
    ROS_INFO("Got points");
//    ROS_INFO("I heard: [%s]", msg->data.c_str());
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "lidar_points_listener");
    ros::NodeHandle n;

    ros::Subscriber sub = n.subscribe("scene/lidar_points", 1000, processLidarPoints);

    ros::spin();

    return 0;
}
