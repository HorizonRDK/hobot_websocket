# Copyright (c) 2021 Horizon Robotics.All Rights Reserved.

# The material in this file is confidential and contains trade secrets
# of Horizon Robotics Inc. This is proprietary information owned by
# Horizon Robotics Inc. No part of this work may be disclosed,
# reproduced, copied, transmitted, or used in any way for any purpose,
# without the express written permission of Horizon Robotics Inc.

import os
import stat
import signal

from ament_index_python.packages import get_package_prefix
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    # 启动webserver服务
    name = 'nginx'
    for line in os.popen("ps ax | grep " + name + " | grep -v grep"):
        fields = line.split()
        # extracting Process ID from the output
        pid = fields[0]
        # terminating process
        os.kill(int(pid), signal.SIGKILL)

    webserver_path = os.path.join(get_package_prefix('websocket'),
                                  "lib/websocket/webservice")
    os.chdir(webserver_path)
    nginx = "./sbin/" + name
    os.chmod(nginx, stat.S_IRWXU)
    webserver = nginx + " -p ."
    os.system(webserver)

    # 切换到mono2d_body_detection目录
    mono2d_body_detection_path = os.path.join(
        get_package_prefix('mono2d_body_detection'),
        "lib/mono2d_body_detection")
    os.chdir(mono2d_body_detection_path)

    return LaunchDescription([
        # 启动图片发布pkg
        Node(
            package='mipi_cam',
            executable='mipi_cam',
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
            arguments=['--ros-args', '--log-level', 'error']
        ),
        # 启动web展示端
        Node(
            package='websocket',
            executable='websocket',
            output='screen',
            parameters=[
                {"image_topic": "/image_jpeg"},
                {"image_type": "mjpeg"},
                {"only_show_image": False},
                {"smart_topic": "/hobot_mono2d_body_detection"}
            ],
            arguments=['--ros-args', '--log-level', 'error']
        )
    ])
