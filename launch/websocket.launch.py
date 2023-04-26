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
import subprocess

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_prefix


def generate_launch_description():
    # 启动webserver服务
    name = 'nginx'
    nginx = "./sbin/" + name
    webserver = nginx + " -p ."
    launch_webserver = True
    # 查询进程列表，获取所有包含 webserver 字符串的进程
    processes = subprocess.check_output(['ps', 'ax'], universal_newlines=True)
    processes = [p.strip() for p in processes.split('\n') if webserver in p]

    # 如果有进程，说明目标程序已经在运行
    if len(processes) > 0:
        launch_webserver = False

    if launch_webserver:
        print("launch webserver")
        pwd_path = os.getcwd()
        print("pwd_path is ", pwd_path)
        webserver_path = os.path.join(get_package_prefix('websocket'),
                                      "lib/websocket/webservice")
        print("webserver_path is ", webserver_path)
        os.chdir(webserver_path)
        # os.chmod(nginx, stat.S_IRWXU)
        print("launch webserver cmd is ", webserver)
        os.system(webserver)
        os.chdir(pwd_path)
    else:
        print("webserver has launch")

    return LaunchDescription([
        DeclareLaunchArgument(
            'websocket_image_topic',
            default_value='/image_jpeg',
            description='image subscribe topic name'),
        DeclareLaunchArgument(
            'websocket_image_type',
            default_value='mjpeg',
            description='image type'),
        DeclareLaunchArgument(
            'websocket_only_show_image',
            default_value='False',
            description='only show image'),
        DeclareLaunchArgument(
            'websocket_output_fps',
            default_value='0',
            description='output fps'),
        DeclareLaunchArgument(
            'websocket_smart_topic',
            default_value='/hobot_mono2d_body_detection',
            description='smart message subscribe topic name'),
        Node(
            package='websocket',
            executable='websocket',
            output='screen',
            parameters=[
                {"image_topic": LaunchConfiguration('websocket_image_topic')},
                {"image_type": LaunchConfiguration('websocket_image_type')},
                {"only_show_image": LaunchConfiguration(
                    'websocket_only_show_image')},
                {"output_fps": LaunchConfiguration('websocket_output_fps')},
                {"smart_topic": LaunchConfiguration('websocket_smart_topic')}
            ],
            arguments=['--ros-args', '--log-level', 'info']
        )
    ])
