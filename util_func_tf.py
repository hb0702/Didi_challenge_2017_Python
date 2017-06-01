import numpy as np
import mayavi.mlab
from mpl_toolkits.mplot3d import Axes3D
import tensorflow as tf 




def cylindrical_projection_with_labels(lidar,
                                       gt_box3d,
                                       ver_fov = (-24.4, 2.),#(-24.9, 2.), 
                                       hor_fov = (-42.,42.), 
                                       v_res = 0.42,
                                       h_res = 0.33):
    '''
    lidar: a numpy array of shape N*D, D>=3
    ver_fov : angle range of vertical projection in degree
    hor_fov: angle range of horizantal projection in degree
    v_res : vertical resolusion
    h_res : horizontal resolution
    d_max : maximun range distance
    
    return : cylindrical projection (or panorama view) of lidar
    '''
    
    # the input lidar should only contain the point in the front view x = lidar[;,0] > 0

    # x = lidar[:,0]
    # y = lidar[:,1]
    # z = lidar[:,2]
    x, y, z = tf.split(lidar, [1,1,1], 1) 

    #d = np.sqrt(np.square(x)+np.square(y))
    d = tf.sqrt(tf.square(x) + tf.square(y))

    # theta = np.arctan2(-y, x)
    # phi = -np.arctan2(z, d)
    theta = tf.atan(tf.divide(-y,x))
    phi = -tf.atan(tf.divide(z,d))
    # todo : define function atan2 for tensorflow


    x_view = np.int16(np.ceil((theta*180/np.pi - hor_fov[0])/h_res))
    y_view = np.int16(np.ceil((phi*180/np.pi + ver_fov[1])/v_res))



    
    #x_max = int(np.ceil((hor_fov[1] - hor_fov[0])/h_res))
    #y_max = int(np.ceil((ver_fov[1] - ver_fov[0])/v_res))
    x_max = 255
    y_max = 63
    
    indices = np.logical_and( np.logical_and(x_view >= 0, x_view <= x_max), 
                           np.logical_and(y_view >= 0, y_view <= y_max)  )
    
    x_view = x_view[indices]
    y_view = y_view[indices]
    z = z[indices]
    d = d[indices]
    d_z = [[d[i],z[i]] for i in range(len(d))]
    
    view = np.zeros([y_max+1, x_max+1, 2],dtype=np.float64)
    view[y_view,x_view] = d_z
    
    encode_boxes = np.array([box_encoder(lidar[i], gt_box3d) for i in range(len(lidar))])
    encode_boxes = encode_boxes[indices]
    
    box = np.zeros([y_max+1, x_max+1, 10],dtype=np.float32)
    box[y_view,x_view] = encode_boxes
    
    # seg = np.zeros([y_max+1, x_max+1],dtype=np.float32)
    # seg[y_view,x_view] = encode_boxes[:,0]
    # seg = seg.reshape(y_max+1, x_max+1,1)
    
    # reg = np.zeros([y_max+1, x_max+1, 7],dtype=np.float32)
    # reg[y_view,x_view] = encode_boxes[:,1:]
    
    return view, box