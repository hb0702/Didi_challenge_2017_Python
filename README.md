![alt text][logo]
[logo]: ./1494587465307.jpg "Team Korea"

# DiDi Challenge

Udacity launched a first-of-its kind competition with Didi Chuxing. The focus will be on a core feature of self-driving cars the Automated Safety and Awareness Processing Stack (ASAPS), which identifies stationary and moving objects from a moving car, using data that includes Velodyne point cloud, radar objects, and camera image frames.

## Model Pipeline

The flow of our code is follwing

##### 0. Ros_script : using ROS, extract data from .bag file. Extracted files are lidar, images, radar, and gps sensor data.


##### 1. Convert_kiti_to_numpy.py : For training data, we used kiti dataset which included lidar, and vehicle track information in tracklet xml format. We converted all data into numpy in python.
    
    
##### 2. Convert_kiti_to_panorama.py : Our DeepLearning model was trained to take in panoramic presentation of 3D lidar points, thus lidar was converted into a picture like format.
    
    
##### 3. Train.py (Fully_conv_model_for_lidar.py) : Model was trained in FCN using Tensorflow as backend. GPU were used to accerlate the training. Train.py produce a model wieght file in h5 format.
##### More detail of the model is in [model_README](./model_README.md)


##### 4. Generate_tracklet.ipynb : The model weight that was produced in train.py was used to predict the location of cars in lidar coordinate. This then was turned into the tracklet xml format for the submission.