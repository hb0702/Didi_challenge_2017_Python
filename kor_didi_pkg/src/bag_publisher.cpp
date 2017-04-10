#include <ros/ros.h>
#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <sensor_msgs/Image.h>
#include <velodyne_msgs/VelodyneScan.h>
#include "std_msgs/String.h"
#include <boost/foreach.hpp>
#include <ros/package.h>

int main(int argc, char **argv)
{
    ros::init(argc, argv, "publish_scene");

    std::string fileName(argv[1]);
    ros::NodeHandle n;

    ros::Publisher lidarPublisher = n.advertise<velodyne_msgs::VelodyneScan>("/velodyne_packets", 1);
    ros::Publisher imagePublisher = n.advertise<sensor_msgs::Image>("scene/image", 1);
    ros::Rate loop_rate(10);

    while (ros::ok())
    {
        rosbag::Bag bag;
        bag.open(fileName, rosbag::bagmode::Read);

        std::string bagImageTopic = "/image_raw";
        std::string bagLidarTopic = "/velodyne_packets";

        std::vector<std::string> topics;
        topics.push_back(bagImageTopic);
        topics.push_back(bagLidarTopic);

        rosbag::View view(bag, rosbag::TopicQuery(topics));

        BOOST_FOREACH(rosbag::MessageInstance const m, view)
        {
            if (m.getTopic() == bagImageTopic)
            {
                sensor_msgs::Image::ConstPtr imagePtr = m.instantiate<sensor_msgs::Image>();
                if (imagePtr != NULL)
                    imagePublisher.publish(*imagePtr);
            }

            if (m.getTopic() == bagLidarTopic)
            {
                velodyne_msgs::VelodyneScan::ConstPtr lidarPointPtr = m.instantiate<velodyne_msgs::VelodyneScan>();
                if (lidarPointPtr != NULL)
                    lidarPublisher.publish(*lidarPointPtr);
            }
        }

        bag.close();
        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}
