# ROS Installtion
1. uBuntu 20.04
2. ros noetic
 - `sudo apt-get install python3-rosdep2 python3-rosinstall-generator python3-vcstools python3-vcstool build-essential`
 - `sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'`
 - `sudo apt install curl`
 - `curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | sudo apt-key add -`
 - `sudo apt update`
 - `sudo apt install ros-noetic-desktop-full`
 - `echo "source /opt/ros/noetic/setup.bash" >> ~/.bashrc`
 - `sudo apt install python3-rosdep`
 - `sudo rosdep init`
 - `rosdep update`
# Package Dependency Setup
 - `sudo apt-get install ros-noetic-navigation`
## Build CppLinuxSerial
 - `git clone https://github.com/gbmhunter/CppLinuxSerial.git`
 - `git switch tag/v2.5.0`
 - `cd CppLinuxSerial`
 - `mkdir build`
 - `cmake ..`
 - `make, sudo make install`
# Build
 - `git clone https://github.com/ThatAquarel/snowplower_mono.git`
 - `cd drive_controller_ws/`
 - `catkin_make`
 - `catkin_make install`
# Start Node
 - cp binary file to install folder
 - roscore start server
 - rosrun hw_base hw_node to start the node
# Issues
1. Built binary in the devel folder, not in install folder
`./devel/lib/hw_base/hw_node`
2. The serial point is fixed for /dev/ttyUSB0