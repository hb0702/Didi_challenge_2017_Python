#!/usr/bin/env python
import sys
import os
import rospy as rp
import numpy as np
from sensor_msgs.msg import PointCloud2
import sensor_msgs.point_cloud2 as pc2

class point_extractor:

	def __init__(self, input_file):
		self.point_subscriber = rp.Subscriber("/velodyne_points", PointCloud2, self.on_points_received)
		self.output_folder = os.path.splitext(input_file)[0] + '/'
		if not os.path.exists(self.output_folder):
			os.makedirs(self.output_folder)
		self.output_cnt = 0
	
	def on_points_received(self, data):
		#rp.loginfo(rp.get_caller_id() + " Point received")
		points = []
		for p in pc2.read_points(data, skip_nans=True, field_names=("x", "y", "z")):
			points.append([p[0], p[1], p[2]])
		points = np.array(points)
		#rp.loginfo("-- Shape: %s", str(points.shape))
		#rp.loginfo("-- %s", self.output_folder + 'lidar_' + str(self.output_cnt) + '.npy')
		np.save(self.output_folder + '/lidar_' + str(self.output_cnt) + '.npy', points)
		self.output_cnt += 1

def listen():
	extractor = point_extractor(input_file = sys.argv[1])
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

