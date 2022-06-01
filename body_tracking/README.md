# 功能介绍

body_tracking app功能为控制机器人跟随人体运动。

订阅智能结果ai_msgs，运行策略，确定跟随track及移动目的位置。

通过发布消息直接控制机器人旋转和平移运动。

详细的控制说明如下：

## 被跟随人体的选择

当启用唤醒手势时，识别到唤醒手势后，通过判断做手势的人手框是否在人体框内来确定跟随的人体。因此要求做唤醒手势时，人手需要在人体框内。

当未启用唤醒手势时，选择人体检测框宽度最大的人体作为被跟随人体。

已有跟随人体的情况下，其他人体不会触发机器人控制。

只有当被跟随人体消失时，才会重新寻找新的跟随人体。连续track_serial_lost_num_thr帧未检测到人体，判断人体消失，支持启动和运行时动态配置。

## 唤醒手势

唤醒手势用于唤醒机器人跟随人体的功能。

当启用唤醒手势时，机器人会跟随做了唤醒手势的人体。一般用于人较多，环境复杂的场景，通过启用唤醒手势避免误触发控制功能。

当未启用唤醒手势时，不会触发机器人跟随功能。

使用"OK"手势作为唤醒跟随手势，手势动作举例如下：

![image-ok](images/image-ok.png)

使用"Palm"手势作为取消跟随手势，取消后需要重新使用唤醒手势选择跟随人体。手势动作举例如下：

![image-palm](images/image-palm.png)

## 控制策略

已找到被跟随人体后，对于每一帧输入的智能结果处理策略如下：

判断人体检测框中心点和机器人之间的角度，角度超过阈值（activate_robot_rotate_thr，支持启动和运行时动态配置）时，控制机器人旋转，保持被跟随人体在机器人正前方。

当被跟随人体消失时，停止机器人运动，并寻找新的被跟随人体。

当跟随人体在机器人正前方时，判断人体检测框上边界（检测框的top坐标），超过阈值（activate_robot_move_thr，支持启动和运行时动态配置）时，控制机器人运动。

# 使用介绍

## 依赖

### 硬件依赖

具备小R机器人小车，包括安装了X3开发板（X3 sdb或者X3 Pi）和camera传感器（USB或MIPI camera）。

launch启动文件默认配置使用的是USB camera。

### 软件依赖

1、系统

X3开发板（X3 sdb或者X3 Pi）安装了X3 Ubuntu系统。

2、TogetherROS部署包

使用all_build.sh配置脚本（完整编译模式）配置编译出来的TogetherROS部署包install，部署包中包含运行此APP所需的以下package：

- mipi_cam package：发布图片msg
- hobot_codec package：jpeg图片编码&发布
- mono2d_body_detection package：发布人体、人头、人脸、人手框感知msg
- hand_lmk_detection package：发布人手关键点感知msg
- hand_gesture_detection package：发布手势识别结果msg
- websocket package：渲染图片和ai感知msg
- body_tracking package：人体跟随
- xrrobot package：小R机器人小车运动控制驱动


## 运行

将TogetherROS部署包install拷贝到地平线X3开发板上（如果是在X3上编译，忽略拷贝步骤），并执行如下命令运行：

### **Ubuntu**

启动检测算法和人体跟随pkg：

```
export COLCON_CURRENT_PREFIX=./install
source ./install/setup.bash
# config中为示例使用的模型，根据实际安装路径进行拷贝
# 如果是板端编译（无--merge-install编译选项），拷贝命令为cp -r install/PKG_NAME/lib/PKG_NAME/config/ .，其中PKG_NAME为具体的package名。
cp -r install/lib/mono2d_body_detection/config/ .
cp -r install/lib/hand_lmk_detection/config/ .
cp -r install/lib/hand_gesture_detection/config/ .

#启动launch文件，文件中配置使能了激活手势
ros2 launch install/share/hobot_app_xrrobot_body_tracking/launch/hobot_app_xrrobot_body_tracking.launch.py
```



## 注意事项

1. 此APP只支持运行在X3 Ubuntu系统。
1. 板端使用launch启动，需要安装依赖，安装命令：`pip3 install lark-parser`。设备上只需要配置一次，断电重启不需要重新配置。
2. 启动小车运动pkg，需要配置驱动：`cp install/lib/xrrobot/config/58-xrdev.rules /etc/udev/rules.d/`，拷贝后重启X3开发板。设备上只需要配置一次，断电重启不需要重新配置。
3. 第一次运行web展示需要启动webserver服务，运行方法为:

- cd 到websocket的部署路径下：`cd install/lib/websocket/webservice/`（如果是板端编译（无--merge-install编译选项）执行命令为`cd install/websocket/lib/websocket/webservice`）
- 启动nginx：`chmod +x ./sbin/nginx && ./sbin/nginx -p .`
- 设备重启需要重新配置。

# 结果分析

## X3结果展示

```

[body_tracking-7] [WARN] [1653430533.523069034] [ParametersClass]: TrackCfg param are
[body_tracking-7] activate_wakeup_gesture: 0
[body_tracking-7] track_serial_lost_num_thr: 100
[body_tracking-7] activate_robot_rotate_thr: 45
[body_tracking-7] activate_robot_move_thr: 5
[body_tracking-7] move_step: 0.3
[body_tracking-7] rotate_step: 0.5
[body_tracking-7] img_width: 960
[body_tracking-7] img_height: 544
[body_tracking-7] 
[body_tracking-7] [WARN] [1653430533.712812076] [TrackingManager]: update frame_ts 395787, 873
[body_tracking-7] [WARN] [1653430533.713105576] [TrackingManager]: Tracking body start!, track_id: 1, frame_ts: 395787, tracking_sta(0:INITING, 1:TRACKING, 2:LOST): 1, gesture: 0
[body_tracking-7] [WARN] [1653430535.018442618] [TrackingManager]: Do move! body_rect_width: 353, thr: 864, move_step_ratio: 1, body_rect_to_top: 20, img_height: 544, move_step: 0.3
[body_tracking-7] [WARN] [1653430535.220268535] [TrackingManager]: Do rotate move, ts sec: 3397, nanosec: 387800000
[body_tracking-7] [WARN] [1653430535.220408576] [RobotCmdVelNode]: RobotCtl, angular: 0 0 0, linear: 0.3 0 0, pub twist ts: 1653430535220394

```

以上log截取了部分app通过launch文件启动后的输出。启动后先打印相关配置（TrackCfg param）。由于launch文件中配置不启用手势激活功能，检测到人体后小车就开始进入跟随状态（tracking_sta值为1），并以0.3m/s的速度前进运动（RobotCtl, angular: 0 0 0, linear: 0.3 0 0）靠近人体。

## web效果展示



# 常见问题

1、Ubuntu下运行启动命令报错`-bash: ros2: command not found`

当前终端未设置ROS2环境，执行命令配置环境：

```
export COLCON_CURRENT_PREFIX=./install
source ./install/setup.bash
```

在当前终端执行ros2命令确认当前终端环境是否生效：

```
# ros2
usage: ros2 [-h] Call `ros2 <command> -h` for more detailed usage. ...

ros2 is an extensible command-line tool for ROS 2.

optional arguments:
  -h, --help            show this help message and exit
```

如果输出以上信息，说明ros2环境配置成功。

注意！对于每个新打开的终端，都需要重新设置ROS2环境。

2、小车不运动

2.1 检查小车运动控制pkg是否启动成功

重新开启一个终端，执行top命令查看是否有xrrobot进程，如果无，确认/etc/udev/rules.d/58-xrdev.rules配置文件是否存在，如果不存在，按照“使用介绍”章节的“注意事项”进行配置。

2.2 检查是否检测到人体

查看输出log中“tracking_sta”关键字值是否为1。

2.3 向小车发布运动控制命令

重新开启一个终端（仅对Ubuntu系统有效），执行命令控制小车转动：`ros2 topic pub -r 10 /cmd_vel geometry_msgs/Twist '{linear: {x: 0, y: 0, z: 0}, angular: {x: 0, y: 0, z: 0.1}}'`，用于确认小车运动是否正常。如果小车不转动，检查小车的运动控制模块。

3、终端无log信息输出

3.1 确认launch文件中的node是否都启动成功

重新开启一个终端（仅对Ubuntu系统有效），执行top命令查看launch文件中的node进程是否都在运行，否则使用ros2 run命令单独启动相关node确认启动失败原因。

3.2 查看每个node是否都有发布msg

根据launch文件中每个node配置的发布和订阅的topic名，使用ros2 topic echo（仅对Ubuntu系统有效）命令显示每个topic是否有消息发布，如果无，再确认没有发布的原因。

注意！如果运行ros2 topic命令失败，执行命令安装依赖：`pip3 install netifaces`
