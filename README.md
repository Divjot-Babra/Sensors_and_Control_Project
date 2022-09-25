# Sensors and Control Project
Project for controlling a Fetch robot for following a marker pattern.

### INSTRUCTIONS FOR INSTALLING REPOSITORY

##### These instructions assume that ROS Melodic has been installed and a catkin workspace has already been created within the home directory.

Open a terminal window and type the following commands:
```
sudo apt-get install ros-melodic-visp
sudo apt-get install ros-melodic-robot-controllers
```

For installing Fetch simulator:
```
sudo apt-get update
sudo apt-get install ros-melodic-fetch-gazebo-demo
```

For installing Turtlebot robot:
```
cd ~/catkin_ws/src/  
git clone https://github.com/ROBOTIS-GIT/turtlebot3_msgs.git  
git clone https://github.com/ROBOTIS-GIT/turtlebot3.git  
git clone https://github.com/ROBOTIS-GIT/turtlebot3_simulations.git  
cd ~/catkin_ws
catkin_make
source devel/setup.bash
```

Open the .bashrc file:
```
gedit ~/.bashrc
```

Add the following lines to your .bashrc:
```
export GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH:~/catkin_ws/src/fetch_gazebo/fetch_gazebo/models/gazebo_models_worlds_collection/models
export GAZEBO_RESOURCE_PATH=$GAZEBO_RESOURCE_PATH:~/catkin_ws/src/fetch_gazebo/fetch_gazebo/models/gazebo_models_worlds_collection/worlds
export TURTLEBOT3_MODEL=waffle
```

Save and close the .bashrc file and then type the following line into a terminal window:
```
source ~/.bashrc
```

For installing Fetch robot:
```
sudo apt install ros-melodic-fetch-calibration ros-melodic-fetch-open-auto-dock ros-melodic-fetch-navigation ros-melodic-fetch-tools -y
cd ~/catkin_ws/src/ 
git clone https://github.com/fetchrobotics/fetch_gazebo.git    
cd ~/catkin_ws   
catkin_make   
source devel/setup.bash
```

For installing vision_visp ROS package:
```
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
```

Clone the Git repository into the *Home* directory using the following SSH URL:
```
cd ~/
git clone git@github.com:Divjot-Babra/Sensors_and_Control_Project.git
```

In order to run the simulator, the following files from the repository must be copied to the correct directories within the catkin workspace:
#### !!! If asked, make sure to replace the files in their respective directories. !!!
```
cd ~/Sensors_and_Control_Project/project_files/
cp fetch.launch.xml ~/catkin_ws/src/fetch_gazebo/fetch_gazebo/launch/include/
cp myLaunch.launch ~/catkin_ws/src/fetch_gazebo/fetch_gazebo/launch/
cp myWorld.sdf ~/catkin_ws/src/fetch_gazebo/fetch_gazebo/worlds/ 
cd ~/catkin_ws/src/turtlebot3/turtlebot3_description/meshes/ 
mkdir markers
cd ~/Sensors_and_Control_Project/project_files/
cp QrCodeCube.dae ~/catkin_ws/src/turtlebot3/turtlebot3_description/meshes/markers/
cp QrCodeCube.png ~/catkin_ws/src/turtlebot3/turtlebot3_description/meshes/markers/
cp tracklive_usb.launch ~/catkin_ws/src/vision_visp/visp_auto_tracker/launch/
cp turtlebot3_waffle.urdf.xacro ~/catkin_ws/src/turtlebot3/turtlebot3_description/urdf/
```

Build the catkin workspace:
```
cd ~/catkin_ws/
catkin_make
```

To start the simulator: 
```
roslaunch fetch_gazebo myLaunch.launch
```

#### --- END OF INSTRUCTIONS ---
