import numpy as np
import os
import cv2

from kitti_data import pykitti
from kitti_data.pykitti.tracklet import parseXML, TRUNC_IN_IMAGE, TRUNC_TRUNCATED
from kitti_data.draw import *
from kitti_data.io import *


## objs to gt boxes ##
def obj_to_gt_boxes3d(objs):

    num        = len(objs)
    gt_boxes3d = np.zeros((num,8,3),dtype=np.float32)
    gt_labels  = np.zeros((num),    dtype=np.int32)

    for n in range(num):
        obj = objs[n]
        b   = obj.box
        label = 1 #<todo>

        gt_labels [n]=label
        gt_boxes3d[n]=b

    return  gt_boxes3d, gt_labels

## extract rgb images
def make_rgb(dataset, directory = './extract_kiti/rgb'):
    if not os.path.exists(directory):
        os.makedirs(directory)
        
    num_frames=len(dataset.velo)
    for n in range(num_frames):
        print('rgb', n)
        rgb = dataset.rgb[n][0]
        rgb =(rgb*255).astype(np.uint8)
        rgb = cv2.cvtColor(rgb, cv2.COLOR_RGB2BGR)
        cv2.imwrite(directory + '/rgb_' + str(n) + '.png', rgb)

## extract lidar
def make_lidar(dataset, directory = './extract_kiti/lidar'):
    if not os.path.exists(directory):
        os.makedirs(directory)
    
    num_frames=len(dataset.velo)
    for n in range(num_frames):
        print('lidar', n)
        lidar = dataset.velo[n]
        np.save(directory + '/lidar_' + str(n) + '.npy',lidar)

## extract labels
def make_gt_labels(dataset, dir_3dbox = './extract_kiti/gt_boxes3d', dir_rgb_label = './extract_kiti/gt_labels'):
    if not os.path.exists(dir_3dbox):
        os.makedirs(dir_3dbox)
        
    if not os.path.exists(dir_rgb_label):
        os.makedirs(dir_rgb_label)

    num_frames=len(dataset.velo)
    for n in range(num_frames):
        print('gt_boxes3d', n)
        objs = objects[n]
        gt_boxes3d, gt_labels = obj_to_gt_boxes3d(objs)

        np.save(dir_3dbox +  '/gt_boxes3d_' + str(n) + '.npy',gt_boxes3d)
        np.save(dir_rgb_label + '/gt_labels_' + str(n) + '.npy',gt_labels)

if __name__ == '__main__':
	basedir = '/home/minh/Documents/Online_courses/Didi_challenge/data/KITTI_data/2011_09_26'
	date = '2011_09_26'
	drive = '0001'

	# The range argument is optional - default is None, which loads the whole dataset
	dataset = pykitti.raw(basedir, date, drive) #, range(0, 50, 5))

	# Load some data
	dataset.load_calib()         # Calibration data are accessible as named tuples
	dataset.load_timestamps()    # Timestamps are parsed into datetime objects
	dataset.load_oxts()          # OXTS packets are loaded as named tuples
	dataset.load_gray()         # Left/right images are accessible as named tuples
	dataset.load_rgb()          # Left/right images are accessible as named tuples
	dataset.load_velo()          # Each scan is a Nx4 array of [x,y,z,reflectance]

	tracklet_file = '/home/minh/Documents/Online_courses/Didi_challenge/data/KITTI_data/2011_09_26/2011_09_26/tracklet_labels.xml'

	num_frames=len(dataset.velo)
	objects = read_objects(tracklet_file, num_frames, findCarOnly=True)


	if not os.path.exists('./extract_kiti'):
		os.makedirs('./extract_kiti')

	make_rgb(dataset)
	make_lidar(dataset)
	make_gt_labels(dataset)
