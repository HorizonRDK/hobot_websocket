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

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch_ros.actions import Node
from launch.substitutions import TextSubstitution
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_prefix

"""
功能：
检查设备上是否已启动nginx服务，
如果未启动，启动websocket package中的nginx服务后程序退出，
如果已启动，程序直接退出。


使用：
使用方法1，直接启动此launch文件：
ros2 launch websocket hobot_websocket_service.launch.py
运行结束后，程序退出。


使用方法2，在另一个launch文件中引用：
1、创建include description
    web_service_launch_include = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('websocket'),
                'launch/hobot_websocket_service.launch.py'))
    )

2、将创建include description作为node添加到LaunchDescription
    return LaunchDescription([
        web_service_launch_include,
        user_defined_node
    ])
"""

def generate_launch_description():
    # 启动webserver服务
    name = 'nginx'
    launch_webserver = True
    for line in os.popen("ps ax | grep " + name + " | grep -v grep"):
        launch_webserver = False
        break

    if launch_webserver:
        print("launch webserver")
        webserver_path = os.path.join(get_package_prefix('websocket'),
                                    "lib/websocket/webservice")
        print("webserver_path is ", webserver_path)
        os.chdir(webserver_path)
        nginx = "./sbin/" + name
        os.chmod(nginx, stat.S_IRWXU)
        webserver = nginx + " -p ."
        print("launch webserver cmd is ", webserver)
        os.system(webserver)
    else:
        print("webserver has launch")

    return LaunchDescription([
            ])