# Sensors and Control Project
Project for controlling a Fetch robot for following a guider robot.

To start the simulator, type the following command into a terminal: 
roslaunch fetch_gazebo myLaunch.launch 

Terminal Commands:
sudo apt-get install ros-melodic-visp
sudo apt-get install ros-melodic-root-controllers

-----Add the following lines to your .bashrc (type: gedit .bashrc)-----

export GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH:~/catkin_ws/src/fetch_gazebo/fetch_gazebo/models/gazebo_models_worlds_collection/models
export GAZEBO_RESOURCE_PATH=$GAZEBO_RESOURCE_PATH:~/catkin_ws/src/fetch_gazebo/fetch_gazebo/models/gazebo_models_worlds_collection/worlds
export TURTLEBOT3_MODEL=waffle 

Then:
source ~/.bashrc
