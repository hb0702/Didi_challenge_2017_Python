import numpy as np
import tensorflow as tf
import keras
from keras.models import load_model
import os

from sklearn.cluster import DBSCAN

from keras.optimizers import Adam

from cluster_classify_model import cluster_classify_model
from cluster_classify_util import *
from cluster_classify_train import *
from test_on_udacity_data import *

from keras.utils.generic_utils import get_custom_objects
get_custom_objects().update({"my_loss": my_loss})


def predict(model,lidar, thresh=0.5):
    lidar, labels = cluster(lidar)
    list_clusters = list(set(labels))
    nb_clusters = len(list_clusters)
    
    list_of_cluster = np.array([lidar[labels == list_clusters[i]] for i in range(nb_clusters)] )
    
    centers = np.zeros((nb_clusters,2))
    imgs = np.zeros((nb_clusters, 64,64,2))
    
    for i in range(nb_clusters):
        img, center = discretize(list_of_cluster[i])
        imgs[i] = img
        centers[i] = center

    features = model.predict(imgs)
    thresh_features = features[features[:,0] >= thresh]
    centers = centers[features[:,0] >= thresh]
    boxes = np.array([gt_box_decode(features[i], centers[i], z_min = -1.5) for i in range(len(features)) ])
    
    return boxes

if __name__ == "__main__":
	
	lidar = np.load('./data/training_didi_data/car_train_edited/bmw_sitting_still/lidar/lidar_100.npy')
	gtbox = np.load('./data/training_didi_data/car_train_gt_box_edited/bmw_sitting_still/gt_boxes3d/gt_boxes3d_100.npy')
	viz_mayavi_with_labels(lidar, gtbox)


	model = load_model('./saved_model/model_for_car_classifier_30_June_10_199.h5') 
	boxes = predict(model, lidar)

	viz_mayavi_with_labels(lidar, boxes)

