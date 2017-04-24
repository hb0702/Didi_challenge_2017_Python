# Didi challenge

### How to run `train.py`
Store kitti dataset in `data_dir` (line 112 file `train.py`). This folder is orgalnized as follows:

`data_dir`
+ 0001
    + gt_boxes3d
        + `gt_boxes3d_0.npy`
        + `gt_boxes3d_1.npy`
        + ...
    + lidar
        + `lidar_0.npy`
        + `lidar_1.npy`
        + ...
+ 0002
    + ...

The function `list_of_data()` (lines 13-26) creates a two lists. One contains names of  lidar files and other contains names of ground true boxes.

The function `train_batch_generator()` creates a generator for training.

The trained weight will be stored as `./saved_model/model.h5`.

### The model
Please  refer to the paper `Vehicle Detection from 3D Lidar Using Fully Convolutional Network` for the detail.

The ideal of the model as follows:

`Input` : tensor of shape `(64,256,2)`, which is the 84 degree panoramic view of lidar, contains the depth of height information of lidar point.

`Output:` two tensors:
+ A tensor of shape `(64,256,1)` where the number at the position `[x,y,0]` presents the at which probability the the pixel `[x,y]` belongs the the obstacle. This part works as a segmentation problem (`out1` line 110, file `fully_conv_model_for_lidar.py`)
+ A tensor of shape `(64,256,9)` where the 9 value vector `[x,y,:]` presents the coordinate of three corners of the bounding box containing the point `[x,y]`. I chose the corners number 0, 1, 6 of the box, the other five corners will be inferred from the three corners. This part works as a multi-regression problem (`out2` line 118, file `fully_conv_model_for_lidar.py`)

`Loss function`: function `my_loss()` line 88-107 file `train.py`. The total loss includes two losses: log loss for segmentation part (`seg_loss`) and mean squared error (MSE) for regression part.

### Todo
1. Combine the predicted boxes to output a single box for each obstacle (Non-Maximum Suppression, clustering, heatmap, ...)
2. Modify the regression part of the model to get better accuracy
3. Normalize the data before training
4. Modify the loss function: add weights for two losses based on the statistics of training data
