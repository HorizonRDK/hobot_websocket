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

import os
import stat
import signal
import subprocess

from ament_index_python.packages import get_package_prefix
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    # 启动webserver服务
    name = 'nginx'
    nginx = "./sbin/" + name
    webserver = nginx + " -p ."

    # 查询进程列表，获取所有包含 webserver 字符串的进程
    processes = subprocess.check_output(['ps', 'ax'], universal_newlines=True)
    processes = [p.strip() for p in processes.split('\n') if webserver in p]

    if len(processes) > 0:
        pid = int(processes[0].split()[0])
        subprocess.run(['kill', str(pid)])

    webserver_path = os.path.join(get_package_prefix('websocket'),
                                  "lib/websocket/webservice")
    os.chdir(webserver_path)
    # os.chmod(nginx, stat.S_IRWXU)
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
                {"output_fps": 10},
                {"smart_topic": "/hobot_mono2d_body_detection"}
            ],
            arguments=['--ros-args', '--log-level', 'info']
        )
    ])
