#!/usr/bin/env python
import rospy as rp
import numpy as np
from sensor_msgs.msg import PointCloud2
import sensor_msgs.point_cloud2 as pc2
import copy

class point_extractor:

	def __init__(self):
		self.point_subscriber = rp.Subscriber("/velodyne_points", PointCloud2, self.on_points_received, 100)
	
	def on_points_received(self, data):
		rp.loginfo(rp.get_caller_id() + " Point received")
		points = pc2.read_points(data, skip_nans=True)
		rp.loginfo("-- Shape: %s", str(points.shape)

def listen():
	extractor = point_extractor()
	# In ROS, nodes are uniquely named. If two nodes with the same
	# node are launched, the previous one is kicked off. The
	# anonymous=True flag means that rospy will choose a unique
	# name for our 'listener' node so that multiple listeners can
	# run simultaneously.
	rp.init_node('point_extractor', anonymous=True)
	# spin() simply keeps python from exiting until this node is stopped
	rp.spin()

if __name__ == '__main__':
	listen()
