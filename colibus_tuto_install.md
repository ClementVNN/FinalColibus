# Installation Guide for Colibus Simulator

This guide provides instructions to install all dependencies and set up the Colibus Simulator.

This tutorial assumes that you are working from your home directory. If you are working from a different directory, please replace `~/` with the current path to your  directory (You can find the current path by running the command `pwd`. For example, if you are working from the Document dir, `pwd`will return `~/Document/`).

---

## Installation Steps

To install all the dependencies and the simulator, you can run the following command in the terminal:

```bash
locale

# Update and install locales
sudo apt update && sudo apt install locales &&
sudo locale-gen en_US en_US.UTF-8 &&
sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8 &&
export LANG=en_US.UTF-8

# Verify locale settings
locale

sudo apt install software-properties-common -y &&
sudo add-apt-repository universe

sudo apt update && sudo apt install curl -y &&
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg

# Add ROS repository to sources list
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null

# Update and install ROS 2 Humble Desktop
sudo apt update &&
sudo apt upgrade -y &&
sudo apt install ros-humble-desktop -y

# Install rosdep and initialize it
sudo apt update &&
sudo apt install -y python3-rosdep &&
sudo rosdep init &&
rosdep update

# Source ROS 2 setup.bash in .bashr`
source /opt/ros/humble/setup.bash
echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc

# Install colcon and Gazebo
sudo apt update &&
sudo apt install -y python3-colcon-common-extensions

sudo apt update && 
sudo apt-get install ros-humble-ros-gz 
sudo apt install gazebo -y
```
------------------------------
### Clone the Colibus Repository

Run the following command to clone the Colibus repository:

```bash
git clone https://github.com/ClementVNN/FinalColibus.git 
```

------------------------------


Follow these steps to replace the placeholder path in the `CMakeLists.txt` file with the actual path to your `colibus_main` directory.

---

### Identify the Current Directory Path

First, determine the full path to your `colibus_main` directory. Run the following command in your terminal:

```bash
pwd
````

Then replace the placeholder path in the `CMakeLists.txt` file with the actual path to your `colibus_main` directory. Run the following command in your terminal:

```bash
sed -i 's|/home/ubuntu/FinalColibus/colibus-main|/home/ubuntu/colibus-main|g' /home/ubuntu/FinalColibus/colibus-main/src/colibus_simulator/CMakeLists.txt
```

-----------------------------

### Build and Install the Colibus Simulator

After running the above commands, you can install the Colibus Simulator by running the following commands:

Verify that you are running from the `colibus_main` directory:

```bash
colcon build --symlink-install
```

You will find that the simulator is installed in the `install` directory.
You can verify the installation by running the following command:

```bash
source install/setup.bash
ros2 pkg list | grep colibus
```


To finish this part, you can add the following line to your `.bashrc` file:

```bash
echo "source install/setup.bash" >> ~/.bashrc
```

--------------------------

### *When we say in another terminal, it means it has to be sourced with the command below, and we assume you are working from the `colibus_main` dir* : 
```
source install/setup.bash 
```
OR

```
source ~/.bashrc
```
------------------------------------

### Running the Simulator : 

#### LIDAR + Camera: 

```
ros2 launch colibus_simulator launch_sim.launch.py worlds:=/src/colibus_simulator/worlds/obstacles.world
```

(Optional) To control the robot with the keyboard, open a new terminal and run the following command:
```
ros2 launch teleop_twist_keyboard teleop_twist_keyboard_launch.py
```

In rviz : \
Robot Model : /robot_description \
Laser Scan : /scan (Size : 0.05) \
Image : /camera/image_raw \
Camera : /camera/image_raw 


---------------------------------------------------------------
#### SLAM (Simultaneous Localization and Mapping) : 

```
sudo apt install ros-humble-slam-toolbox -y 
```

```
ros2 launch colibus_simulator online_async_slam.launch.py use_sim_time:=True 
```

*If not working, try to run the following command*: 
```
ros2 launch slam_toolbox online_async_slam.launch.py use_sim_time:=True
```

(Optional) To control the robot with the keyboard, open a new terminal and run the following command:
```
ros2 launch teleop_twist_keyboard teleop_twist_keyboard_launch.py
```

In rviz : \
Let previous topic remains \
Add Topic Map : /map 

**(Optional)** To save the map, run ros2 service list and find the service /slam_toolbox/map_saver_service. Then, run the following command: ros2 service call /slam_toolbox/map_saver_service slam_toolbox_msgs/srv/SaveMap
or in rviz, click on the Panel option (above) and select the SlamToolBox Plugin 


-----------------------------------------------------------------------
#### Navigation : 

```
sudo apt install ros-humble-navigation2 -y &&
sudo apt install ros-humble-nav2-bringup -y &&
sudo apt install ros-humble-twist-mux -y 
```

```
ros2 run twist_mux twist_mux --ros-args --params-file ./src/colibus_simulator/config/twist_mux.yaml -r cmd_vel_out:=diff_cont/cmd_vel_unstamped 
```


In a new terminal, run the following command: 
```
ros2 launch colibus_simulator launch_sim.launch.py worlds:=/src/colibus_simulator/worlds/obstacles.world
```


In another terminal, run the following command:

```
ros2 launch slam_toolbox online_async_slam.launch.py params_file:=./src/colibus_simulator/config use_sim_time:=True
```

In another terminal, run :

```
ros2 launch nav2_bringup navigation_launch.py use_sim_time:=True map:=/src/colibus_simulator/maps/mapper_params_online_async.yaml 
```


In rviz : \
Let previous Topic remains \
Add Topic : \
Topic Map : /global_costmap/costmap \
Updated Topic Map : /global_costmap/costmap
Color Scheme : costmap

Click on 2D global Pose in Rviz, and with the mouse select a place, the robot will go to it. 

----------------------------------

Good Luck !