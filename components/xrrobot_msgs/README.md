## package name

xrrobot 

## package功能

小R机器人 msg，目前只使用到IMU msg，包括加速度计和陀螺仪。

xrrobot发布/raw_imu话题，供上层应用做数据处理和机器人定位。

## package编译

`colcon build --packages-select xrrobot xrrobot_msgs`

## 运行package

```
export COLCON_CURRENT_PREFIX=./install
source ./install/local_setup.sh
ros2 run xrrobot xrrobot
```
