import numpy as np
import mayavi.mlab
from util_func import rotation

def viz_mayavi_with_labels(points, boxes, view_boxes = True, vals="distance"):
    x = points[:, 0]  # x position of point
    y = points[:, 1]  # y position of point
    z = points[:, 2]  # z position of pointfrom mpl_toolkits.mplot3d import Axes3D
    # r = lidar[:, 3]  # reflectance value of point
    d = np.sqrt(x ** 2 + y ** 2)  # Map Distance from sensor

    # Plot using mayavi -Much faster and smoother than matplotlib
    #import mayavi.mlab
    if vals == "height":
        col = z
    else:
        col = d

    fig = mayavi.mlab.figure(bgcolor=(0, 0, 0), size=(640, 360))
    mayavi.mlab.points3d(x, y, z,
                         col,          # Values used for Color
                         mode="point",
                         colormap='spectral', # 'bone', 'copper', 'gnuplot'
                         # color=(0, 1, 0),   # Used a fixed (r,g,b) instead
                         figure=fig,
                         )
    
    if view_boxes:
        for i in range(len(boxes)):
            car = boxes[i]
            x = car[:,0]
            y = car[:,1]
            z = car[:,2]

            mayavi.mlab.plot3d(x[:4], y[:4], z[:4], tube_radius=0.025)#, colormap='Spectral')
            mayavi.mlab.plot3d(x[[0,3]], y[[0,3]], z[[0,3]], tube_radius=0.025)
            mayavi.mlab.plot3d(x[[0,4]], y[[0,4]], z[[0,4]], tube_radius=0.025)
            mayavi.mlab.plot3d(x[[1,5]], y[[1,5]], z[[1,5]], tube_radius=0.025)
            mayavi.mlab.plot3d(x[[2,6]], y[[2,6]], z[[2,6]], tube_radius=0.025)
            mayavi.mlab.plot3d(x[[3,7]], y[[3,7]], z[[3,7]], tube_radius=0.025)


            mayavi.mlab.plot3d(x[-4:], y[-4:], z[-4:], tube_radius=0.025)#, colormap='Spectral')
            mayavi.mlab.plot3d(x[[4,7]], y[[4,7]], z[[4,7]], tube_radius=0.025)
        
    mayavi.mlab.show()


def cylindrical_projection_v2(lidar, 
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
    
    x = lidar[:,0]
    y = lidar[:,1]
    z = lidar[:,2]
    d = np.sqrt(np.square(x)+np.square(y))
    
    
    theta = np.arctan2(-y, x)
    phi = -np.arctan2(z, d)
    
    
    
    x_view = np.int16(np.ceil((theta*180/np.pi - hor_fov[0])/h_res))
    y_view = np.int16(np.ceil((phi*180/np.pi + ver_fov[1])/v_res))
    
    x_view_2 = np.int16(np.ceil((hor_fov[1] - theta*180/np.pi)/h_res))
    y_view_2 = np.int16(np.ceil((-phi*180/np.pi - ver_fov[0])/v_res))
    
    x_max = np.int16(np.ceil((hor_fov[1] - hor_fov[0])/h_res))
    y_max = np.int16(np.ceil((ver_fov[1] - ver_fov[0])/v_res))
    
    
   
    indices = np.logical_and( np.logical_and(x_view >= 0, x_view <= x_max), 
                          np.logical_and(y_view >= 0, y_view <= y_max)  )
    
    x_view = x_view[indices]
    y_view = y_view[indices]
    z = z[indices]
    d = d[indices]
    d_z = [[d[i],z[i]] for i in range(len(d))]
    
    view = np.zeros([y_max+1, x_max+1, 2],dtype=np.float32)
    view[y_view,x_view] = d_z
    return view

def cylindrical_projection_v3(lidar, 
                           ver_fov = (-24.4, 2.),#(-24.9, 2.), 
                           #hor_fov = (-42.,42.), 
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
    #num_points = len(lidar)
    x = lidar[:,0]
    y = lidar[:,1]
    z = lidar[:,2]
    
    d = np.sqrt(np.square(x)+np.square(y))
    
    #theta = np.zeros_like(x)
    #phi = np.zeros_like(x)
    
    theta = np.arctan2(-y, x)
    phi = -np.arctan2(z, d)
       
    
    x_view = np.int16(np.ceil((theta*180/np.pi + 180)/h_res))
    y_view = np.int16(np.ceil((phi*180/np.pi + ver_fov[1])/v_res))
    
    x_max = np.int16(np.ceil(360/h_res))
    y_max = np.int16(np.ceil((ver_fov[1] - ver_fov[0])/v_res))
    
    
    #print(np.)
    #x_max = np.max(x_view)
    #y_max = np.max(y_view)
    #print('x_min, x_max, y_min, y_max', np.min(x_view), x_max, np.min(y_view), y_max)
    
    indices = np.logical_and( np.logical_and(x_view >= 0, x_view <= x_max), 
                          np.logical_and(y_view >= 0, y_view <= y_max)  )
    
    x_view = x_view[indices]
    y_view = y_view[indices]
    z = z[indices]
    d = d[indices]
    d_z = [[d[i],z[i]] for i in range(len(d))]
    
    view = np.zeros([y_max+1, x_max+1, 2],dtype=np.float32)
    view[y_view,x_view] = d_z
    
    out = np.zeros([y_max+1, 2*x_max+2, 2],dtype=np.float32)
    middle = int((x_max+1)/2)
    out[:,:middle,:] = view[:, middle:2*middle,:]
    out[:,middle:middle + x_max +1, :] = view
    out[:, middle+x_max+1:, :] = view[:,:middle,:]
    return out

def cylindrical_projection_for_test_ver2(lidar, 
                                         ver_fov = (-24.4, 2.),#(-24.9, 2.), 
                                         #hor_fov = (-42.,42.), 
                                         v_res = 0.42,
                                         h_res = 0.33,
                                         angle_offset = 90):
                                       #d_max = None):
    '''
    lidar: a numpy array of shape N*D, D>=3
    ver_fov : angle range of vertical projection in degree
    hor_fov: angle range of horizantal projection in degree
    v_res : vertical resolusion
    h_res : horizontal resolution
    d_max : maximun range distance
    
    return : cylindrical projection (or panorama view) of lidar
    '''
    
    x = lidar[:,0]
    y = lidar[:,1]
    z = lidar[:,2]
    
    d = np.sqrt(np.square(x)+np.square(y))
    
    #theta = np.zeros_like(x)
    #phi = np.zeros_like(x)
    
    theta = np.arctan2(-y, x)
    phi = -np.arctan2(z, d)
       
    
    x_view = np.int16(np.ceil((theta*180/np.pi + 180)/h_res))
    y_view = np.int16(np.ceil((phi*180/np.pi + ver_fov[1])/v_res))
    
    x_max = np.int16(np.ceil(360/h_res))
    y_max = np.int16(np.ceil((ver_fov[1] - ver_fov[0])/v_res))
    
    
    #print(np.)
    #x_max = np.max(x_view)
    #y_max = np.max(y_view)
    #print('x_min, x_max, y_min, y_max', np.min(x_view), x_max, np.min(y_view), y_max)
    
    indices = np.logical_and( np.logical_and(x_view >= 0, x_view <= x_max), 
                          np.logical_and(y_view >= 0, y_view <= y_max)  )
    
    x_view = x_view[indices]
    y_view = y_view[indices]
    x = x[indices]
    y = y[indices]
    z = z[indices]
    d = d[indices]
    
    theta = theta[indices]
    phi = phi[indices]
    coord = [[x[i],y[i],z[i],theta[i],phi[i],d[i]] for i in range(len(x))]
    
    view = np.zeros([y_max+1, x_max+1, 6],dtype=np.float32)
    view[y_view,x_view] = coord
    
    pad = int(angle_offset*(x_max + 1)/360)
    
    out = np.zeros([y_max+1, x_max+1+2*pad, 6],dtype=np.float32)
    
    #middle = int((x_max+1)/2)
    out[:,:pad,:] = view[:, -pad:,:]
    out[:,pad:pad+x_max+1, :] = view
    out[:, pad+x_max+1:x_max+1+2*pad, :] = view[:,:pad,:]
    return out

def predict_boxes_ver2(model,lidar, cluster = True, seg_thres=0.5, cluster_dist = 0.1, min_dist = 1.5, neigbor_thres = 3):
    
    view =  cylindrical_projection_for_test_ver2(lidar)
    #print('view.shape: ',view.shape)
    width = view.shape[1]
    
    cylindrical_view = view[:,:,[5,2]].reshape(1,64,width,2)
    step = int(width/256)
    #print('total_step = ', step )
    
    list_boxes = []
    for i in range(step):
        pred = model.predict(cylindrical_view[:,:,i*256:(i+1)*256,:])
        pred = pred[0]


        pred = pred.reshape(-1,8)
        view = view.reshape(-1,6)
        thres_pred = pred[pred[:,0] > seg_thres]
        thres_view = view[pred[:,0] > seg_thres]
        
        num_boxes = len(thres_pred)
        boxes = np.zeros((num_boxes,8,3))
        for i in range(num_boxes):
            boxes[i,0] = thres_view[i,:3] - rotation(thres_view[i,3],thres_pred[i,1:4])
            boxes[i,6] = thres_view[i,:3] - rotation(thres_view[i,3],thres_pred[i,4:7])

            boxes[i,2,:2] = boxes[i,6,:2]
            boxes[i,2,2] = boxes[i,0,2]

            phi = thres_pred[i,-1]

            z = boxes[i,2] - boxes[i,0]
            boxes[i,1,0] = (np.cos(phi)*z[0] + np.sin(phi)*z[1])*np.cos(phi) + boxes[i,0,0]
            boxes[i,1,1] = (-np.sin(phi)*z[0] + np.cos(phi)*z[1])*np.cos(phi) + boxes[i,0,1]
            boxes[i,1,2] = boxes[i,0,2]

            boxes[i,3] = boxes[i,0] + boxes[i,2] - boxes[i,1]
            boxes[i,4] = boxes[i,0] + boxes[i,6] - boxes[i,2]
            boxes[i,5] = boxes[i,1] + boxes[i,4] - boxes[i,0]
            boxes[i,7] = boxes[i,4] + boxes[i,6] - boxes[i,5]
        list_boxes.append(boxes)
    
#     for i in range(len(list_boxes)-1):
#         list_boxes[0] = np.concatenate([list_boxes[0], list_boxes[i+1]], axis = 0)
    boxes = np.concatenate(list_boxes, axis = 0)
    print(boxes.shape)
    if not cluster:
        return boxes
    
    
    box_dist = np.zeros((len(boxes), len(boxes)))
    all_boxes = np.copy(boxes)

    flatteb_boxes = boxes.reshape(-1,24)
    for i in range(num_boxes):
        box_dist[i] = np.sqrt(np.sum(np.square(flatteb_boxes[[i]] - flatteb_boxes), axis = 1))
        
    cluster_boxes = []

    thres_box_dist = box_dist < cluster_dist
    neighbor = np.sum(thres_box_dist, axis = 1)

    while len(neighbor)>0:

        ind = np.argmax(neighbor)

        if neighbor[ind] < neigbor_thres:
            break

        cluster_boxes.append(boxes[ind])

        remain_indx = box_dist[ind] > min_dist

        box_dist = box_dist[remain_indx]
        box_dist = box_dist[:,remain_indx]

        thres_box_dist = thres_box_dist[remain_indx]
        thres_box_dist = thres_box_dist[:,remain_indx]

        boxes = boxes[remain_indx]

        neighbor = np.sum(thres_box_dist, axis = 1)

    return all_boxes, np.array(cluster_boxes) 

def predict_boxes_ver3(model,lidar, num_views = 4, cluster = True, seg_thres=0.5, cluster_dist = 0.2, min_dist = 2, neigbor_thres = 5):
    
    all_view =  cylindrical_projection_for_test_ver2(lidar)
    #print('view.shape: ',view.shape)
    width = all_view.shape[1]
    
    cylindrical_view = all_view[:,:,[5,2]].reshape(1,64,width,2)
    
    
    # if num_views = 4:
    #     step = 4
    #     centers = [int(width/2), int(2*width/3), int(width/3), int(5*width/6)]
    # else:
    #     step = 6
    #     centers = [int(width/2), int(2*width/3), int(width/3), int(5*width/6)]
    # #print('total_step = ', step )

    step = width*4/(6*num_views)

    list_boxes = []
    for i in range(num_views):
        center = int(width/6 + i*step)

        pred = model.predict(cylindrical_view[:,:,center-128: center+128,:])
        pred = pred[0]


        pred = pred.reshape(-1,8)
        view = all_view[:,center-128: center+128,:].reshape(-1,6)
        thres_pred = pred[pred[:,0] > seg_thres]
        thres_view = view[pred[:,0] > seg_thres]
        
        num_boxes = len(thres_pred)
        
        boxes = np.zeros((num_boxes,8,3))
        for i in range(num_boxes):
            boxes[i,0] = thres_view[i,:3] - rotation(thres_view[i,3],thres_pred[i,1:4])
            boxes[i,6] = thres_view[i,:3] - rotation(thres_view[i,3],thres_pred[i,4:7])

            boxes[i,2,:2] = boxes[i,6,:2]
            boxes[i,2,2] = boxes[i,0,2]

            phi = thres_pred[i,-1]

            z = boxes[i,2] - boxes[i,0]
            boxes[i,1,0] = (np.cos(phi)*z[0] + np.sin(phi)*z[1])*np.cos(phi) + boxes[i,0,0]
            boxes[i,1,1] = (-np.sin(phi)*z[0] + np.cos(phi)*z[1])*np.cos(phi) + boxes[i,0,1]
            boxes[i,1,2] = boxes[i,0,2]

            boxes[i,3] = boxes[i,0] + boxes[i,2] - boxes[i,1]
            boxes[i,4] = boxes[i,0] + boxes[i,6] - boxes[i,2]
            boxes[i,5] = boxes[i,1] + boxes[i,4] - boxes[i,0]
            boxes[i,7] = boxes[i,4] + boxes[i,6] - boxes[i,5]
        list_boxes.append(boxes)
    
#     for i in range(len(list_boxes)-1):
#         list_boxes[0] = np.concatenate([list_boxes[0], list_boxes[i+1]], axis = 0)
    boxes = np.concatenate(list_boxes, axis = 0)
    #print(boxes.shape)
    if not cluster:
        return boxes
    
    else:
        box_cluster = box_clustering(boxes, cluster_dist, min_dist, neigbor_thres)

    return boxes, box_cluster 

def box_clustering(boxes, cluster_dist = 0.2, min_dist = 2, neigbor_thres = 5):
    num_boxes = len(boxes)
    box_dist = np.zeros((num_boxes, num_boxes))
    
    flatteb_boxes = boxes.reshape(-1,24)
    for i in range(num_boxes):
        box_dist[i] = np.sqrt(np.sum(np.square(flatteb_boxes[[i]] - flatteb_boxes), axis = 1))
        
    cluster_boxes = []

    thres_box_dist = box_dist < cluster_dist
    neighbor = np.sum(thres_box_dist, axis = 1)

    while len(neighbor)>0:

        ind = np.argmax(neighbor)

        if neighbor[ind] < neigbor_thres:
            break

        cluster_boxes.append(boxes[ind])

        remain_indx = box_dist[ind] > min_dist

        box_dist = box_dist[remain_indx]
        box_dist = box_dist[:,remain_indx]

        thres_box_dist = thres_box_dist[remain_indx]
        thres_box_dist = thres_box_dist[:,remain_indx]

        boxes = boxes[remain_indx]

        neighbor = np.sum(thres_box_dist, axis = 1)

    return np.array(cluster_boxes) 

def clustering_one_boxes(boxes, cluster_dist, min_dist = 1.5, neigbor_thres = 3):
    num_boxes = len(boxes)
    box_dist = np.zeros((num_boxes, num_boxes))
    all_boxes = np.copy(boxes)

    flatteb_boxes = boxes.reshape(-1,24)
    for i in range(num_boxes):
        box_dist[i] = np.sqrt(np.sum(np.square(flatteb_boxes[[i]] - flatteb_boxes), axis = 1))
        
    cluster_boxes = []

    thres_box_dist = box_dist < cluster_dist
    neighbor = np.sum(thres_box_dist, axis = 1)

    ind = np.argmax(neighbor)

    if neighbor[ind] < neigbor_thres:
        return []
    else:
        return boxes[box_dist[ind]<neigbor_thres]

def convert_box_to_sphere(boxes):
    if len(boxes) == 0:
        return boxes
    centers = (boxes[:,0,:] + boxes[:,6:,:])/2
    dims = sp.sqrt(np.sum(np.square(boxes[:,0,:] - boxes[:,6:,:]),axis = 1))
    return np.mean(centers, axis=0), np.mean(dims)/2

def predict_boxes_view(model,all_view, num_views = 4, cluster = True, seg_thres=0.5, cluster_dist = 0.2, min_dist = 2, neigbor_thres = 5):
    #print('view.shape: ',view.shape)
    width = all_view.shape[1]
    
    cylindrical_view = all_view[:,:,[5,2]].reshape(1,64,width,2)
    
    
    step = width*4/(6*num_views)

    list_boxes = []
    for i in range(num_views):
        center = int(width/6 + i*step)

        pred = model.predict(cylindrical_view[:,:,center-128: center+128,:])
        pred = pred[0]


        pred = pred.reshape(-1,8)
        view = all_view[:,center-128: center+128,:].reshape(-1,6)
        thres_pred = pred[pred[:,0] > seg_thres]
        thres_view = view[pred[:,0] > seg_thres]
        
        num_boxes = len(thres_pred)
        
        boxes = np.zeros((num_boxes,8,3))
        for i in range(num_boxes):
            boxes[i,0] = thres_view[i,:3] - rotation(thres_view[i,3],thres_pred[i,1:4])
            boxes[i,6] = thres_view[i,:3] - rotation(thres_view[i,3],thres_pred[i,4:7])

            boxes[i,2,:2] = boxes[i,6,:2]
            boxes[i,2,2] = boxes[i,0,2]

            phi = thres_pred[i,-1]

            z = boxes[i,2] - boxes[i,0]
            boxes[i,1,0] = (np.cos(phi)*z[0] + np.sin(phi)*z[1])*np.cos(phi) + boxes[i,0,0]
            boxes[i,1,1] = (-np.sin(phi)*z[0] + np.cos(phi)*z[1])*np.cos(phi) + boxes[i,0,1]
            boxes[i,1,2] = boxes[i,0,2]

            boxes[i,3] = boxes[i,0] + boxes[i,2] - boxes[i,1]
            boxes[i,4] = boxes[i,0] + boxes[i,6] - boxes[i,2]
            boxes[i,5] = boxes[i,1] + boxes[i,4] - boxes[i,0]
            boxes[i,7] = boxes[i,4] + boxes[i,6] - boxes[i,5]
        list_boxes.append(boxes)
    
#     for i in range(len(list_boxes)-1):
#         list_boxes[0] = np.concatenate([list_boxes[0], list_boxes[i+1]], axis = 0)
    boxes = np.concatenate(list_boxes, axis = 0)
    #print(boxes.shape)
    if not cluster:
        return boxes
    
    else:
        box_cluster = box_clustering(boxes, cluster_dist, min_dist, neigbor_thres)

    return boxes, box_cluster 

def vertical_shift(view, step = 1):
    shifted = np.zeros_like(view)
    if step > 0:
        shifted[step:] = view[:-step]
    elif step < 0:
        shifted[:-step] = view[step:]
    return shifted


def max_interpolation(view, num_points = 5):
    h, w, d = view.shape
    list_views = np.zeros((num_points,h,w,d))
    list_views[0] = view
    step = int(num_points/2)
    for i in range(step):
        list_views[2*i+1] = vertical_shift(view, i+1)
        list_views[2*i+2] = vertical_shift(view, -i-1)
    
    max_d = np.argmax(list_views[:,:,:,5], axis = 0)
    out = np.zeros_like(view)
    for i in range(h):
        for j in range(w):
            out[i,j] = list_views[max_d[i,j],i,j]
    return out

