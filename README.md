# Diamond-Age-Coding-Challenge1

Yucheng Yang, Coding Challenge 1

Requirements: Ubuntu 20.04

ROS noetic

catkin workspace

commands to set up the environment:
sudo apt install ros-noetic-libfranka ros-noetic-franka-ros

sudo apt remove "*libfranka*"

sudo apt install build-essential cmake git libpoco-dev libeigen3-dev

git clone --recursive https://github.com/frankaemika/libfranka    ##I only use panda arm robot

cd libfranka

mkdir build

cd build

cmake -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTS=OFF ..

cpack -G DEB

sudo dpkg -i libfranka*.deb

## copy the code and put it under catkin/src

catkin_make

## In some cases, some packages should be installed by yourself just using sudo apt install ros-noetic-*****-****

## The following two lines should be added to .bashrc
source /opt/ros/noetic/setup.bash

source /home/yucheng/catkin_ws/devel/setup.bash

## Run the following commands in two terminals
roslaunch panda_moveit_config demo.launch

rosrun moveit_tutorials pick_place_tutorial


##Reference: 
https://github.com/frankaemika/libfranka
https://ros-planning.github.io/moveit_tutorials/
