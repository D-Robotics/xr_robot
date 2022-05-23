from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # 启动图片发布pkg
        Node(
            package='mipi_cam',
            executable='mipi_cam',
            output='screen',
            parameters=[
                {"out_format": "nv12"},
                {"image_width": 960},
                {"image_height": 544},
                {"io_method": "shared_mem"},
                {"video_device": "F37"}
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
                {"in_mode": "shared_mem"},
                {"in_format": "nv12"},
                {"out_mode": "ros"},
                {"out_format": "jpeg"},
                {"sub_topic": "/hbmem_img"},
                {"pub_topic": "/image_jpeg"}
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
                {"image_topic": "/image_jpeg"},
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
