# ROS packages in Project Venom

## Contents

| package name | purpose |
| --- | --- |
| venom_offb | Nodes that are related to offboard mode control, including navigation decision making. | 
| venom_perception | Sensor nodes that is integrated with ROS. `Class Perceiver ` is designed to wrap different sensors. Note that zed-ros-wrapper should be placed here.|


## Prerequisite

1. [PX4 toolchain](https://dev.px4.io/en/setup/dev_env_linux_ubuntu.html#convenience-bash-scripts): we will not build mavros from scratch. Choose **ubuntu_ros_gazebo.sh** to install. After installation, move `~/src/Firmware` to `~/Documents`.
```
$ mv ~/src/Firmware ~/Documents
$ rmdir ~/src
```

2. [geographiclib](https://github.com/mavlink/mavros/blob/master/mavros/scripts/install_geographiclib_datasets.sh):
```
$ wget https://raw.githubusercontent.com/mavlink/mavros/master/mavros/scripts/install_geographiclib_datasets.sh
$ chmod +x install_geographiclib_datasets.sh
$ sudo ./install_geographiclib_datasets.sh
```

3. mavros: we will use `catkin_make` instead of `rosbuild`. Therefore, remove the directory created by ubuntu_ros_gazebo.sh and install mavros via apt-get.
```
$ rm -rf ~/catkin_make
$ sudo apt-get install ros-kinetic-mavros ros-kinetic-extras
```

4. [CUDA 9.0](https://developer.nvidia.com/cuda-downloads): remember to setup library path.
```
$ echo 'export PATH=/usr/local/cuda-9.0/bin${PATH:+:${PATH}}' >> ~/.bashrc
$ echo 'export LD_LIBRARY_PATH=/usr/local/cuda-9.0/lib64${LD_LIBRARY_PATH:+:${LD_LIBRARY_PATH}}' >> ~/.bashrc
```

5. [ZED SDK 2.2.1](https://www.stereolabs.com/developers/release/2.2/): please choose **CUDA 9 -> ZED SDK for Linux** to download.
```
$ mkdir -p ~/catkin_ws/src
$ unzip zed-ros-wrapper-2.2.x.zip -d ~/catkin_make/src/zed-ros-wrapper-2.2
$ cd ~/catkin_ws
$ catkin_make
```

6. [zed-ros-wrapper](https://github.com/stereolabs/zed-ros-wrapper/releases): please choose v2.2.x download. After download, unzip the file and `catkin_make` it.

## Installation

1. Download this repository
```
$ mkdir -p ~/catkin_ws/src
$ cd ~/catkin_ws/src
$ git clone git@github.com:GitWyd/ProjectVenomROS.git
```

2. Compile
```
$ cd ~/catkin_ws
$ catkin_make
```

## Tutorial

### Software in the loop (SITL) Simulation
1. Let's assume that you have moved `Firmware` to `~/Documents`. To make life easier, let's make an shortcut activating the gazebo simulator.
```
$ echo "alias sitl_gazebo='cd ~/Documents/Firmware; make posix_sitl_default gazebo'" >> ~/.bashrc
$ source ~/.bashrc
$ sitl_gazebo   # This activates the gazebo simulator
```

2. Connect the ZED camera to your USB3.0 port. Then activates utility nodes.
```
$ roslaunch zed_wrapper zed.launch
$ roslaunch venom_offb sitl.launch
```

3. We are ready to take off!! Run this command and you should see the drone hover at height 3.0 meter. Press 'q' to land.
```
$ rosrun venom_offb navigation_node
```
