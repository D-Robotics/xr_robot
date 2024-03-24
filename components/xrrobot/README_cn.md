[English](./README.md) | 简体中文

## package name

xrrobot 

## package功能

小R机器人的驱动程序，完成以下功能：
1. 获取并发布原始传感器数据，比如/raw_odom，/raw_imu
2. 订阅 /cmd_vel 话题，并控制小车运动

## package编译

`colcon build --packages-select xrrobot xrrobot_msgs`

## 运行package

```
export COLCON_CURRENT_PREFIX=./install
source ./install/local_setup.sh
ros2 run xrrobot xrrobot
```
