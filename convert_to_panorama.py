import numpy as np
import os
import time
from multiprocessing import Pool
from multiprocessing import Process


def box_encoder(point, boxes):
    '''
        
    '''
    box_num = in_which_box(point, boxes)
    #print(box_num)
    if box_num==0:
        return np.zeros(8)
        
    
    box = boxes[box_num-1]
    #print(box.shape)
    
    theta = np.arctan2(-point[1], point[0])
    #print(theta*180/np.pi)
    #phi = -np.arctan2(point[2], np.sqrt(point[0]**2 + point[1]**2) )
    u0 = point[:3] - box[0] 
    ru0 = rotation(-theta, u0)
    
    u6 = point[:3] - box[6] 
    ru6 = rotation(-theta, u6)
    
    x = np.sqrt(np.sum(np.square(box[1,:2] - box[2,:2])))
    z = np.sqrt(np.sum(np.square(box[0,:2] - box[2,:2])))
    phi = np.arcsin(x/z)

    return np.array([1, ru0[0], ru0[1], ru0[2], ru6[0], ru6[1], ru6[2], phi])

def rotation(theta, point):
    v = np.sin(theta)
    u = np.cos(theta)
    out = np.copy(point)
    out[0] = u*point[0] + v*point[1]
    out[1] = -v*point[0] + u*point[1]
    return out

def is_in_box(point, box):
    '''
    point: tuple (x,y,z) coordinate
    box: numpy array of shape (8,3)
    return: True or False
    '''
    low = np.min(box[:,2])
    high = np.max(box[:,2])
    if (point[2] >= high) or (point[2]<=low):
        return False
    
    v = point[:2] - box[0,:2]
    v1 = box[1,:2] - box[0,:2]
    v2 = box[3,:2] - box[0,:2]
    
    det1 = v[0]*v2[1] - v[1]*v2[0]
    if det1 == 0:
        return False
    
    det2 = v[0]*v1[1] - v[1]*v1[0]
    if det2 == 0:
        return False
    
    t1 = (v1[0]*v2[1] - v1[1]*v2[0])/det1
    s1 = (v1[0]*v[1] - v1[1]*v[0])/det1
    if (t1<=1) or (s1<=0):
        return False
    
    t2 = (v2[0]*v1[1] - v2[1]*v1[0])/det2
    s2 = (v2[0]*v[1] - v2[1]*v[0])/det2
    if (t2<=1) or (s2<=0):
        return False
    
    return True

def in_which_box(point, boxes):
    '''
    return in which box the given point belongs to, return 0 if the point doesn't belong to any boxes
    '''
    for i in range(len(boxes)):
        if is_in_box(point, boxes[i]):
            return i + 1
    return 0

def cylindrical_projection_for_training(lidar,
                                       gt_box3d,
                                       ver_fov = (-24.4, 2.),#(-24.9, 2.), 
                                       hor_fov = (-47.,47.), 
                                       v_res = 0.42,
                                       h_res = 0.33):
    '''
    lidar: a numpy array of shape N*D, D>=3
    gt_box3d: Ground truth boxes of shape B*8*3 (B : number of boxes)
    ver_fov : angle range of vertical projection in degree
    hor_fov: angle range of horizantal projection in degree
    v_res : vertical resolusion
    h_res : horizontal resolution
    
    return : cylindrical projection (or panorama view) of lidar
    '''
    
    x = lidar[:,0]
    y = lidar[:,1]
    z = lidar[:,2]
    d = np.sqrt(np.square(x)+np.square(y))
    
    
    theta = np.arctan2(-y, x)
    phi = -np.arctan2(z, d)
    
    x_view = np.int16(np.ceil((theta*180/np.pi - hor_fov[0])/h_res))
    y_view = np.int16(np.ceil((phi*180/np.pi + ver_fov[1])/v_res))
    
    x_max = np.int16(np.ceil((hor_fov[1]-hor_fov[0])/h_res))
    y_max = 63
    
    indices = np.logical_and( np.logical_and(x_view >= 0, x_view <= x_max), 
                           np.logical_and(y_view >= 0, y_view <= y_max)  )
    
    x_view = x_view[indices]
    y_view = y_view[indices]
    z = z[indices]
    d = d[indices]
    d_z = [[d[i],z[i]] for i in range(len(d))]
    
    view = np.zeros([y_max+1, x_max+1, 10],dtype=np.float32)
    view[y_view,x_view, :2] = d_z
    
    encode_boxes = np.array([box_encoder(lidar[i], gt_box3d) for i in range(len(lidar))])
    encode_boxes = encode_boxes[indices]
    
    #box = np.zeros([y_max+1, x_max+1, 8],dtype=np.float32)
    view[y_view,x_view, 2:] = encode_boxes
    
    return view

def list_of_paths(lidar_dir, gt_box_dir):
	list_of_lidar = []
	list_of_gtbox = []
	list_of_view = []
	for f in os.listdir(lidar_dir):
		lidar_path = os.path.join(lidar_dir, f, 'lidar')
		gtbox_path = os.path.join(gt_box_dir, f, 'gt_boxes3d')

		#lidar_path = os.path.join(path, 'lidar')
		#gtbox_path = os.path.join(path, 'gt_boxes3d')
		view_path = os.path.join(lidar_dir, f,'view')

		if not os.path.exists(view_path):
			os.makedirs(view_path)
		
		num_files = len(os.listdir(lidar_path))

		lidar = [os.path.join(lidar_path, 'lidar_'+str(i)+'.npy') for i in range(num_files) ]
		gtbox = [os.path.join(gtbox_path, 'gt_boxes3d_'+str(i)+'.npy') for i in range(num_files) ]
		view = [os.path.join(view_path, 'view_'+str(i)+'.npy') for i in range(num_files) ]

		list_of_lidar += lidar
		list_of_gtbox += gtbox
		list_of_view += view
	return list_of_lidar, list_of_gtbox, list_of_view


if __name__ == '__main__':

	lidar_dir = './extract_kiti/'
	gt_box_dir = './new_gt_boxes/all_boxes/'
	list_of_lidar, list_of_gtbox, list_of_view = list_of_paths(lidar_dir, gt_box_dir)
    using_pool = False
	
	def convert(i):
		lidar = np.load(list_of_lidar[i])
		gt_box = np.load(list_of_gtbox[i])

		#print('converting: ' + list_of_view[i])

		view = cylindrical_projection_for_training(lidar, gt_box)
		np.save(list_of_view[i], view)
		
		return i


	start = time.time()
	if using_pool:
        p = Pool(8)
        p.map(convert, np.arange(len(list_of_lidar)))
	else:
        for i in range(len(list_of_lidar)):
            convert(i)
    print('Finish convert - total time = {0}'.format(time.time()-start))



	# start = time.time()
	# print('Start convert {} file'.format(len(list_of_lidar)))
	# for i in range(len(list_of_lidar)):
	# 	lidar = np.load(list_of_lidar[i])
	# 	gt_box = np.load(list_of_gtbox[i])

	# 	print('onverting: ' + list_of_view[i])

	# 	view = cylindrical_projection_for_training(lidar, gt_box)
	# 	np.save(list_of_view[i], view)
	

	
