# DiDi Challenge

Udacity launched a first-of-its kind competition with Didi Chuxing. The focus will be on a core feature of self-driving cars the Automated Safety and Awareness Processing Stack (ASAPS), which identifies stationary and moving objects from a moving car, using data that includes Velodyne point cloud, radar objects, and camera image frames.

### Model Pipeline

The flow of our code is follwing

    1. Convert_kiti_to_numpy.py : For training data, we used kiti dataset which included lidar, and vehicle track information in tracklet xml format. We converted all data into numpy in python.
    
    2. Convert_kiti_to_panorama.py : Our DeepLearning model was trained to take in panoramic presentation of 3D lidar points, thus lidar was converted into a picture like format.
    
    3. Train.py (Fully_conv_model_for_lidar.py) : Model was trained in FCN using Tensorflow as backend. GPU were used to accerlate the training. Train.py produce a model wieght file in h5 format.
    
    4. Generate_tracklet.ipynb : The model weight that was produced in train.py was used to predict the location of cars in lidar coordinate. This then was turned into the trackelt xml format for the submission. 

### Time line

The submission deadlines are as noted below. All start dates start at 12:00 AM PST and end-dates/deadlines end at 11:59 PM PST on the noted dates.

#### March 8 – March 21 — Competitor and Team Registration

Competition platform opens for account and team registration. Competitors can register with Udacity accounts or create a new account. Team leaders can recruit team members through Forum.

#### March 22 – May 31 — Round 1

Data set for Round 1 is released. New user and team registration closes on April 21

#### June 1 – June 4 — Round 1 Evaluation

Top 75 teams will be asked to submit runnable code. Code will be spot-checked to prevent fraudulent submissions of that group the Top 50 qualified teams will progress to next round.

#### June 5 – July 3 — Round 2

Data set for Round 2 is released. Teams will no longer be able to add or remove members after May 21.

#### July 4 – July 11 — Finalist Evaluation

Top teams required to submit identity verification documents and runnable code. Code will be evaluated and output compared against scores on final leaderboard. Top 5 teams will be invited to attend final award ceremony at Udacity headquarters in Mountain View, California

#### Post July 11 — Travel arrangements

5 week break for teams to arrange visas and travel that Udacity will help with

#### Jul 12 — Final Award Ceremony

Top 5 teams present their solutions to a panel of Udacity and DiDi executives and have chance to run their code on Udacity’s self-driving car
