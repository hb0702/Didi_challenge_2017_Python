# DiDi Challenge

Udacity launched a first-of-its kind competition with Didi Chuxing. The focus will be on a core feature of self-driving cars the Automated Safety and Awareness Processing Stack (ASAPS), which identifies stationary and moving objects from a moving car, using data that includes Velodyne point cloud, radar objects, and camera image frames.

### Time line

The submission deadlines are as noted below. All start dates start at 12:00 AM PST and end-dates/deadlines end at 11:59 PM PST on the noted dates.

#### March 8 – March 21 — Competitor and Team Registration

Competition platform opens for account and team registration. Competitors can register with Udacity accounts or create a new account. Team leaders can recruit team members through Forum.

#### March 22 – April 21 — Round 1

Data set for Round 1 is released. New user and team registration closes on April 21

#### April 22 – April 30 — Round 1 Evaluation

Top 75 teams will be asked to submit runnable code. Code will be spot-checked to prevent fraudulent submissions of that group the Top 50 qualified teams will progress to next round.

#### May 1 – May 31 — Round 2

Data set for Round 2 is released. Teams will no longer be able to add or remove members after May 21.

#### Jun 1 – Jun 14 — Finalist Evaluation

Top teams required to submit identity verification documents and runnable code. Code will be evaluated and output compared against scores on final leaderboard. Top 5 teams will be invited to attend final award ceremony at Udacity headquarters in Mountain View, California

#### Jun 15 – July 12 — Travel arrangements

5 week break for teams to arrange visas and travel that Udacity will help with

#### Jul 12 — Final Award Ceremony

Top 5 teams present their solutions to a panel of Udacity and DiDi executives and have chance to run their code on Udacity’s self-driving car

### How to setup environment
* Install python-2.7 or Anaconda-2 (recommended)
* Install ROS

        $ sudo apt-get install ros-indigo-desktop-full

* Setup ROS environment - add following to ~/.bashrc

        source /opt/ros/indigo/setup.bash
		source ~/catkin_ws/devel/setup.bash
		export ROS_MASTER_URI=http://localhost:11311 export ROS_HOSTNAME=localhost

* Install opencv

        $ pip install opencv-python

* Install CUDA and CuDNN
* Install tensorflow-gpu

        $ pip install tensorflow-gpu

* Install keras

        $ pip install keras

### How to setup development environment with Qt
* Go to catkin workspace

        $ cd ~/catkin_ws/src

* Download kor_didi_pkg source
* Move the original file of catkin_ws/src/CMakeLists.txt (should be /opt/ros/indigo/share/catkin/cmake/toplevel.cmake) to catkin_ws/src/, and change the file name to CMakeLists.txt
* Setup Qt Creator permission - add following to ~/.bashrc

        sudo -s chmod o+w /home/parkjaeil0108/.config/QtProject/qtcreator/*.xml
        sudo chown -R $USER:$USER /home/parkjaeil0108/.config/QtProject/
        sudo -s chmod o+w /home/parkjaeil0108/catkin_ws/src/kor_didi_pkg/*.*

* Open Qt Creator

        $ qtcreator

* Open catkin_ws/src/CMakeLists.txt on Qt Creator, set build directory to ~/catkin_ws/build

### How to run kor_didi_pkg
* Go to catkin workspace

        $ cd ~/catkin_ws/src

* Download kor_didi_pkg source

* Go to catkin workspacee

        $ cd ~/catkin_ws
* Build

        $ catkin_make

* Run kor_didi_run.sh with .bag file path

        $ sh '/home/parkjaeil0108/catkin_ws/src/kor_didi_run.sh' '/home/parkjaeil0108/challenge/Didi-Training-Release-1/approach_1.bag'

### How to extract points(.npy) from .bag file
* Go to catkin workspace source

        $ cd ~/catkin_ws/src

* Download or git clone kor_didi_pkg source

* Go to catkin workspacee

        $ cd ~/catkin_ws
        
* Build

        $ catkin_make

* Run extract_points.sh with .bag file path

        $ sh '/home/parkjaeil0108/catkin_ws/src/extract_points.sh' '/home/parkjaeil0108/challenge/Didi-Training-Release-1/approach_1.bag'

* In the directory .bag file located, (bag file name)/lidar folder will be created, and .npy files will be saved in it.
