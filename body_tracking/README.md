English| [简体中文](./README_cn.md)

# Function Introduction

The body_tracking app is used to control the robot to follow human body movements.

Subscribe to intelligent results in ai_msgs, run strategies, determine the tracking and moving destination positions.

Control robot rotation and translation movements directly by publishing messages.

Detailed control instructions are as follows:

## Selection of the Followed Human Body

When the wake-up gesture is enabled, after recognizing the wake-up gesture, the person to follow is determined by judging whether the bounding box of the hand making the gesture is inside the human body bounding box. Therefore, when making the wake-up gesture, the hand needs to be within the human body bounding box.

When the wake-up gesture is not enabled, the human body with the maximum width of the detection box is chosen as the followed person.

If there is already a followed person, other bodies will not trigger robot control.

Only when the followed person disappears will a new followed person be searched for. If no human body is detected for consecutive track_serial_lost_num_thr frames, it is judged that the person has disappeared, supporting dynamic configuration during startup and runtime.

## Wake-up Gesture

The wake-up gesture is used to activate the function of the robot following a human body.

When the wake-up gesture is enabled, the robot will follow the person making the wake-up gesture. It is generally used in scenarios with many people and complex environments to avoid accidental triggering of control functions by enabling the wake-up gesture.

When the wake-up gesture is not enabled, the robot follow function will not be triggered.

"OK" gesture is used as the wake-up follow gesture, and the gesture action is as follows:

<img src="images/image-ok.png" width="100" height="100"/>

"Palm" gesture is used as the cancel follow gesture, and after canceling, the wake-up gesture needs to be used again to select the followed person. The gesture action is as follows:

<img src="images/image-palm.png" width="100" height="100"/>


## Control Strategy

After the followed person is found, the strategy for processing the intelligent results input for each frame is as follows:

Determine the angle between the center point of the human body detection box and the robot. When the angle exceeds the threshold (activate_robot_rotate_thr, supports dynamic configuration during startup and runtime), control the robot to rotate and keep the followed person in front of the robot.

When the followed person disappears, stop the robot's movement and search for a new followed person.

When the followed person is in front of the robot, if the top border of the human body detection box (top coordinate of the detection box) exceeds the threshold (activate_robot_move_thr, supports dynamic configuration during startup and runtime), control the robot's movement.

# User Guide

## Dependencies### Hardware Dependencies

Requires the small R robot car, including the X3 development board (X3 sdb or X3 Pi) and camera sensor (USB or MIPI camera) installed.

The default configuration of the launch startup file uses a USB camera.

### Software Dependencies

1. System

X3 development board (X3 sdb or X3 Pi) is running X3 Ubuntu system.

2. TogetherROS Deployment Package

Using the all_build.sh configuration script (full compilation mode) to compile the TogetherROS deployment package install, which includes the following packages required to run this APP:

- mipi_cam package: publishes image msg
- hobot_codec package: encodes & publishes jpeg images
- mono2d_body_detection package: perceives body, head, face, and hand bounding box msg
- hand_lmk_detection package: perceives hand key points msg
- hand_gesture_detection package: publishes gesture recognition results msg
- websocket package: renders images and ai perception msg
- body_tracking package: body tracking
- xrrobot package: controls motion of small R robot car


## Execution

Copy the TogetherROS deployment package install to the Horizon X3 development board (ignore copying steps if compiling on X3) and run the following commands:

### **Ubuntu**

Start detection algorithms and body tracking pkg:

```
export COLCON_CURRENT_PREFIX=./install
source ./install/setup.bash
# config contains example models, copy according to actual installation path
# If compiling on the board (no --merge-install compilation option), the copy command is cp -r install/PKG_NAME/lib/PKG_NAME/config/ ., where PKG_NAME is the specific package name.
cp -r install/lib/mono2d_body_detection/config/ .
cp -r install/lib/hand_lmk_detection/config/ .
cp -r install/lib/hand_gesture_detection/config/ .

# Start the launch file, which enables gesture activation as configured in the file
ros2 launch install/share/hobot_app_xrrobot_body_tracking/launch/hobot_app_xrrobot_body_tracking.launch.py
```## Notes

1. This APP is only compatible with X3 Ubuntu system.
2. Launch is used to start at the board end, and dependencies need to be installed with the command: `pip3 install lark-parser`. Configuration is required only once on the device and does not need to be reconfigured after power off and reboot.
3. To start the car motion pkg, driver configuration is needed: `cp install/lib/xrrobot/config/58-xrdev.rules /etc/udev/rules.d/`, copy it and restart the X3 development board. Configuration is required only once on the device and does not need to be reconfigured after power off and reboot.
4. To run the web display for the first time, the webserver service needs to be started, and the steps are as follows:

- Navigate to the deployment path of the websocket: `cd install/lib/websocket/webservice/` (If it is compiled on the board end (without the --merge-install compilation option), the command to run is `cd install/websocket/lib/websocket/webservice`)
- Start nginx: `chmod +x ./sbin/nginx && ./sbin/nginx -p .`
- Reconfiguration is needed after device reboot.

# Result Analysis

## X3 Result Display

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

The above log snippet shows some output after the app is launched via the launch file. Upon starting, relevant configurations are printed first (TrackCfg param). Since the gesture activation feature is not enabled in the launch file, the car enters the follow-up state once a human body is detected (tracking_sta value is 1), and it moves forward at a speed of 0.3m/s (RobotCtl, angular: 0 0 0, linear: 0.3 0 0) to approach the human body.

## Web Effect Display

# FAQ

1. Error reported when running the startup command under Ubuntu: `-bash: ros2: command not found`

The current terminal has not set up the ROS2 environment. Execute the following commands to configure the environment:

```
export COLCON_CURRENT_PREFIX=./install
source ./install/setup.bash
``````
Check whether the current terminal environment is effective by executing the ros2 command in the current terminal:

```
# ros2
usage: ros2 [-h] Call `ros2 <command> -h` for more detailed usage. ...

ros2 is an extensible command-line tool for ROS 2.

optional arguments:
  -h, --help            show this help message and exit
```

If the above information is output, it indicates that the ros2 environment configuration is successful.

Note! For each new terminal opened, the ROS2 environment needs to be set again.

2. Car is not moving

2.1 Check if the pkg for controlling the car's movement is started successfully

Open a new terminal and execute the top command to check if there is an xrrobot process running. If not, confirm if the /etc/udev/rules.d/58-xrdev.rules configuration file exists. If it does not exist, configure it according to the "Notes" section in the "Usage Guide."

2.2 Check if human detection is detected

Check if the value of the "tracking_sta" keyword in the output log is 1.

2.3 Send motion control commands to the car

Open a new terminal (only valid for Ubuntu systems), execute the command to control the car's rotation: `ros2 topic pub -r 10 /cmd_vel geometry_msgs/Twist '{linear: {x: 0, y: 0, z: 0}, angular: {x: 0, y: 0, z: 0.1}}'` to confirm if the car's movement is normal. If the car does not rotate, check the car's motion control module.

3. No log information output in the terminal

3.1 Confirm if all nodes in the launch file are started successfully

Open a new terminal (only valid for Ubuntu systems), execute the top command to check if the node processes in the launch file are all running. If not, use the ros2 run command to start the related nodes separately to confirm the reason for the startup failure.

3.2 Check if each node is publishing messages

Based on the topic names configured for publishing and subscribing in each node in the launch file, use the ros2 topic echo (only valid for Ubuntu systems) command to display if messages are being published for each topic. If not, reconfirm the reasons for not publishing.

Note! If running the ros2 topic command fails, execute the command to install dependencies: `pip3 install netifaces`
```