# Copyright (c) 2022，Horizon Robotics.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # 启动图片发布pkg
        # F37 mipi cam
        # Node(
        #     package='mipi_cam',
        #     executable='mipi_cam',
        #     output='screen',
        #     parameters=[
        #         {"out_format": "nv12"},
        #         {"image_width": 960},
        #         {"image_height": 544},
        #         {"io_method": "shared_mem"},
        #         {"video_device": "F37"}
        #     ],
        #     arguments=['--ros-args', '--log-level', 'error']
        # ),
        # usb cam
        Node(
            package='hobot_usb_cam',
            executable='hobot_usb_cam',
            output='screen',
            parameters=[
                {"image_width": 640},
                {"image_height": 480},
                {"video_device": "/dev/video8"}
            ],
            arguments=['--ros-args', '--log-level', 'error']
        ),
        # 启动jpeg图片编码&发布pkg
        Node(
            package='hobot_codec',
            executable='hobot_codec_republish',
            output='screen',
            parameters=[
                {"channel": 1},
                {"in_mode": "ros"},
                {"in_format": "jpeg"},
                {"out_mode": "shared_mem"},
                {"out_format": "nv12"},
                {"sub_topic": "/image"},
                {"pub_topic": "/hbmem_img"}
            ],
            arguments=['--ros-args', '--log-level', 'error']
        ),
        # 启动单目rgb人体、人头、人脸、人手框和人体关键点检测pkg
        Node(
            package='mono2d_body_detection',
            executable='mono2d_body_detection',
            output='screen',
            parameters=[
                {"ai_msg_pub_topic_name": "/hobot_mono2d_body_detection"}
            ],
            arguments=['--ros-args', '--log-level', 'error']
        ),
        # 启动人手关键点检测pkg
        Node(
            package='hand_lmk_detection',
            executable='hand_lmk_detection',
            output='screen',
            parameters=[
                {"ai_msg_sub_topic_name": "/hobot_mono2d_body_detection"},
                {"ai_msg_pub_topic_name": "/hobot_hand_lmk_detection"}
            ],
            arguments=['--ros-args', '--log-level', 'error']
        ),
        # 启动web展示pkg
        Node(
            package='websocket',
            executable='websocket',
            output='screen',
            parameters=[
                {"image_topic": "/image"},
                {"image_type": "mjpeg"},
                {"smart_topic": "/hobot_hand_gesture_detection"}
            ],
            arguments=['--ros-args', '--log-level', 'error']
        ),
        # 启动手势识别pkg
        Node(
            package='hand_gesture_detection',
            executable='hand_gesture_detection',
            output='screen',
            parameters=[
                {"ai_msg_sub_topic_name": "/hobot_hand_lmk_detection"},
                {"ai_msg_pub_topic_name": "/hobot_hand_gesture_detection"}
            ],
            arguments=['--ros-args', '--log-level', 'error']
        ),
        # 启动手势交互pkg
        Node(
            package='gesture_control',
            executable='gesture_control',
            output='screen',
            parameters=[
                {"ai_msg_sub_topic_name": "/hobot_hand_gesture_detection"},
                {"twist_pub_topic_name": "/cmd_vel"},
                {"activate_wakeup_gesture": 1},
                {"track_serial_lost_num_thr": 100},
                {"move_step": 0.5},
                {"rotate_step": 0.5}
            ],
            arguments=['--ros-args', '--log-level', 'warn']
        ),
        
        # 启动小R机器⼈小⻋pkg
        Node(
            package='xrrobot',
            executable='xrrobot',
            output='screen',
            arguments=['--ros-args', '--log-level', 'info']
        )
    ])
