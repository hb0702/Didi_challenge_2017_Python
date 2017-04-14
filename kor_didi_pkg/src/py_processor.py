#!/usr/bin/env python
import rospy as rp
import numpy as np
from sensor_msgs.msg import Image
import cv2
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import PointCloud2
import sensor_msgs.point_cloud2 as pc2

class image_processor:

    def __init__(self):
        self.image_subscriber = rp.Subscriber("scene/image", Image, self.on_image_received)
        self.bridge = CvBridge()

    def on_image_received(self, data):
        rp.loginfo(rp.get_caller_id() + "Image received")
        try:
            bayer_image = self.bridge.imgmsg_to_cv2(data, "bayer_grbg8")
            rgb_img = cv2.cvtColor(bayer_image, cv2.COLOR_BAYER_GR2RGB)
        except CvBridgeError as e:
            print(e)
        rp.loginfo(rp.get_caller_id() + "Image converted, shape: %s", str(rgb_img.shape))
        # cv2.imwrite('/home/parkjaeil0108/challenge/Didi-Release-2/Data/1/test.jpg', rgb_img)

class lidar_processor:

    def __init__(self):
        self.point_subscriber = rp.Subscriber("/velodyne_points", PointCloud2, self.on_points_received)

    def on_points_received(self, data):
        rp.loginfo(rp.get_caller_id() + "Point received")
        x_pos = []
        y_pos = []
        z_pos = []
        for p in pc2.read_points(data, skip_nans=True):
            x_pos.append(p[0])
            y_pos.append(p[1])
            z_pos.append(p[2])
	rp.loginfo("%d Point converted, p0: %.2f %.2f %.2f", len(x_pos), x_pos[0], y_pos[0], z_pos[0])

def listen():
    ip = image_processor()
    lp = lidar_processor()
    # In ROS, nodes are uniquely named. If two nodes with the same
    # node are launched, the previous one is kicked off. The
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'listener' node so that multiple listeners can
    # run simultaneously.
    rp.init_node('py_processor', anonymous=True)
    # spin() simply keeps python from exiting until this node is stopped
    rp.spin()

if __name__ == '__main__':
    listen()
