# Sensors and Control Project
Project for controlling a Fetch robot for following a guider robot.

========== INSTRUCTIONS FOR INSTALLING REPOSITORY ==========

Open a terminal window and type the following commands:
sudo apt-get install ros-melodic-visp
sudo apt-get install ros-melodic-robot-controllers

--- For installing simulator ---
sudo apt-get update
sudo apt-get install ros-melodic-fetch-gazebo-demo

--- For installing Turtlebot robot ---
cd ~/catkin_ws/src/  
git clone https://github.com/ROBOTIS-GIT/turtlebot3_msgs.git  
git clone https://github.com/ROBOTIS-GIT/turtlebot3.git  
git clone https://github.com/ROBOTIS-GIT/turtlebot3_simulations.git  
cd ~/catkin_ws
catkin_make
source devel/setup.bash

--- Add the following lines to your .bashrc (type: gedit .bashrc) ---
export GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH:~/catkin_ws/src/fetch_gazebo/fetch_gazebo/models/gazebo_models_worlds_collection/models
export GAZEBO_RESOURCE_PATH=$GAZEBO_RESOURCE_PATH:~/catkin_ws/src/fetch_gazebo/fetch_gazebo/models/gazebo_models_worlds_collection/worlds
export TURTLEBOT3_MODEL=waffle 

--- Save and close the .bashrc file and then type the following line into a terminal window ---
source ~/.bashrc

--- For installing Fetch robot ---
sudo apt install ros-melodic-fetch-calibration ros-melodic-fetch-open-auto-dock ros-melodic-fetch-navigation ros-melodic-fetch-tools -y
cd ~/catkin_ws/src/ 
git clone https://github.com/fetchrobotics/fetch_gazebo.git    
cd ~/catkin_ws   
catkin_make   
source devel/setup.bash

--- For installing vision_visp ROS package ---
sudo apt-get install ros-melodic-vision-visp
cd ~/catkin_ws/src
git clone https://github.com/lagadic/vision_visp.git
cd vision_visp
git checkout melodic
cd ~/catkin_ws
sudo rosdep init
rosdep update
rosdep install --from-paths src --ignore-src --rosdistro melodic
cd ~/catkin_ws
catkin_make -j4 -DCMAKE_BUILD_TYPE=Release

--- For inserting files into respective directories ---
...

--- To start the simulator --- 
roslaunch fetch_gazebo myLaunch.launch 

========== END OF INSTRUCTIONS ==========
