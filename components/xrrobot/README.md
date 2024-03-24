English| [简体中文](./README_cn.md)

## Package name

xrrobot

## Package Functionality

Driver program for XrRobot, which accomplishes the following functions:
1. Get and publish raw sensor data, such as /raw_odom, /raw_imu
2. Subscribe to the /cmd_vel topic and control the movement of the robot

## Package Compilation

`colcon build --packages-select xrrobot xrrobot_msgs`

## Running the Package

```
export COLCON_CURRENT_PREFIX=./install
source ./install/local_setup.sh
ros2 run xrrobot xrrobot
```