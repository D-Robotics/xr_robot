English| [简体中文](./README_cn.md)

## package name

xrrobot 

## package functionality

小R机器人 msg, currently only uses IMU msg, including accelerometer and gyroscope.

xrrobot publishes on the /raw_imu topic for higher-level applications to process data and robot localization.

## package compilation

`colcon build --packages-select xrrobot xrrobot_msgs`

## running the package

```
export COLCON_CURRENT_PREFIX=./install
source ./install/local_setup.sh
ros2 run xrrobot xrrobot
```