import numpy as np
import tensorflow as tf
import keras
import os

from keras.optimizers import Adam

from fully_conv_model_for_lidar_2 import fcn_model
from util_func import *



def list_of_data(data_dir):
	list_of_lidar = []
	list_of_gtbox = []
	for f in os.listdir(data_dir):
	    path = os.path.join(data_dir, f)
	    lidar_path = os.path.join(path, 'lidar')
	    gtbox_path = os.path.join(path, 'gt_boxes3d')
	    num_files = len(os.listdir(lidar_path))
	    
	    lidar = [os.path.join(lidar_path, 'lidar_'+str(i)+'.npy') for i in range(num_files) ]
	    gtbox = [os.path.join(gtbox_path, 'gt_boxes3d_'+str(i)+'.npy') for i in range(num_files) ]
	    list_of_lidar += lidar
	    list_of_gtbox += gtbox
	return list_of_lidar, list_of_gtbox

def data_generator(list_of_lidar, list_of_gtbox):
    '''
    input: list_of_lidar, list_of_gtbox
    output: generator of lidar and gtbox
    '''
    n_sample = len(list_of_lidar)
    next_epoch = True

    while True:
        if next_epoch:
            indices = np.arange(n_sample)
            np.random.shuffle(indices)
            yield list_of_lidar[indices[0]], list_of_gtbox[indices[0]]
            ind = 1
            next_epoch = False
        else:
            yield list_of_lidar[indices[0]], list_of_gtbox[indices[0]]
            ind += 1
            if ind >= n_sample:
                next_epoch = True  




def train_batch_generator(list_of_lidar, list_of_gtbox, batch_size = 4, 
                    data_augmentation = True, width = 256, height = 64):
    '''
    '''
    theta_offset_range = 10*np.pi/180
    ind = 0
    for lidar_file, box_file in data_generator(list_of_lidar, list_of_gtbox):
        lidar = np.load(lidar_file)
        gt_box = np.load(box_file)
        
        if ind == 0:
            batch_sample = np.zeros((batch_size, height, width, 2))
            batch_label = np.zeros((batch_size, height, width, 10))
            
        if data_augmentation:
        # Randomly flip the frame
            flip = np.random.randint(1)
            theta = np.random.uniform(low=-theta_offset_range, high=theta_offset_range)

            #lidar, gt_box = augmentation(theta, flip, lidar, gt_box)
            view, box = cylindrical_projection_for_training_with_augmentation(lidar, gt_box, theta, flip)
        
        else:

        	view, box = cylindrical_projection_for_training(lidar, gt_box)

        batch_sample[ind] = view
        batch_label[ind] = box
        
        ind += 1
        
        if ind == batch_size:
            yield batch_sample, batch_label
            
            ind = 0   

def my_loss(y_true, y_pred):

    seg_true,reg_true = tf.split(y_true, [1, 9], 3)
    seg_pred,reg_pred = tf.split(y_pred, [1, 9], 3)
    
    #ratio = 20*h*w/tf.reduce_sum(seg_true)
    #weight1 = ((ratio-1)*seg_true + 1)/ratio
    
        
    seg_loss = -tf.reduce_mean(tf.multiply(seg_true,tf.log(seg_pred+1e-8)) + tf.multiply(1-seg_true,tf.log(1-seg_pred+1e-8)))
    #seg_loss = -tf.reduce_mean(
    #    tf.multiply(tf.multiply(seg_true,tf.log(seg_pred)) + tf.multiply(1-seg_true,tf.log(1-seg_pred)), weight1))
    
    diff = tf.reduce_mean(tf.squared_difference(reg_true, reg_pred), axis=3, keep_dims=True)
    reg_loss = tf.reduce_mean(tf.multiply(seg_true,diff))
    
    #total_loss = reg_loss
    #total_loss = seg_loss
    total_loss = seg_loss + reg_loss
    return total_loss 


if __name__ == '__main__':

	data_dir = './extract_kiti/'

	list_of_lidar, list_of_gtbox = list_of_data(data_dir)

	# test on just one sample
	#list_of_lidar = [list_of_lidar[108]]
	#list_of_gtbox = [list_of_gtbox[108]]

	model = fcn_model(input_shape = (64,256,2), summary = True)
	opt = Adam(lr=1e-5)
	model.compile(optimizer=opt, loss=my_loss)


	#model.fit_generator(generator=train_batch_generator(list_of_lidar, list_of_gtbox, batch_size = 1, data_augmentation = False),
    #                    steps_per_epoch=1,
    #                    epochs=2000)

	model.fit_generator(generator=train_batch_generator(list_of_lidar, list_of_gtbox, batch_size = 1, data_augmentation = False),
                        steps_per_epoch=262,
                        epochs=3)

	model.save("saved_model/model_4.h5")

	# model_json = model.to_json()
	# with open("saved_model/model.json", "w") as json_file:
	# 	json_file.write(model_json)
	# 	# serialize weights to HDF5
	# model.save_weights("saved_model/model.h5")
	# print("Saved model to disk")

	

