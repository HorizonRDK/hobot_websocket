# WEB展示端

# 功能介绍

为了方便预览图像和算法效果，TROS集成了web展示功能，通过网络将图像和算法结果传输到浏览器端，然后进行渲染显示。

# 编译

## 依赖库


ros package：

- hbm_img_msgs
- ai_msgs

hbm_img_msgs为自定义消息格式，用于发布shared memory类型图像数据，定义在hobot_msgs中。

ai_msgs为自定义消息格式，用于发布算法模推理结果，定义在hobot_msgs中。

## 开发环境

- 编程语言: C/C++
- 开发平台: X3/X86
- 系统版本：Ubuntu 20.0.4
- 编译工具链:Linux GCC 9.3.0/Linaro GCC 9.3.0

## 编译

 支持在X3 Ubuntu系统上编译和在PC上使用docker交叉编译两种方式。

### Ubuntu板端编译

1. 编译环境确认 
   - 板端已安装X3 Ubuntu系统。
   - 当前编译终端已设置TogetherROS环境变量：`source PATH/setup.bash`。其中PATH为TogetherROS的安装路径。
   - 已安装ROS2编译工具colcon，安装命令：`pip install -U colcon-common-extensions`
2. 编译

编译命令：`colcon build --packages-select websocket`

### Docker交叉编译

1. 编译环境确认

   - 在docker中编译，并且docker中已经安装好TogetherROS。docker安装、交叉编译说明、TogetherROS编译和部署说明详见机器人开发平台robot_dev_config repo中的README.md。

2. 编译

   - 编译命令：

```
export TARGET_ARCH=aarch64
export TARGET_TRIPLE=aarch64-linux-gnu
export CROSS_COMPILE=/usr/bin/$TARGET_TRIPLE-

colcon build --packages-select websocket \
   --merge-install \
   --cmake-force-configure \
   --cmake-args \
   --no-warn-unused-cli \
   -DCMAKE_TOOLCHAIN_FILE=`pwd`/robot_dev_config/aarch64_toolchainfile.cmake
```

## 注意事项

# 使用介绍

## 依赖

websocket接收图像消息和智能结果消息，根据时间戳进行匹配，然后输出给web端渲染显示，也可单独显示图像。

图像消息支持`sensor_msgs::msg::Image`以及`shared_mem`的`hbm_img_msgs::msg::HbmMsg1080P`类型消息，必须为hobot codec输出的jpeg格式数据。

智能结果消息支持`ai_msgs::msg::PerceptionTargets`类型消息，其中`header.stamp`必须和该结果对应的image消息相同，websocket会使用该字段进行消息匹配，还有智能结果对应的宽高必须要和接收到的图像分辨率一致。

具体依赖的package有：

- mipi_cam：启动mipi cam，发布nv12类型图像消息
- hobot_codec：将mipi_cam发布的nv12图像编码为websocket需要的jpeg格式图像
- mono2d_body_detection：接收nv12格式数据，进行算法推理，发布人体、人头、人脸、人手框感知消息

## 参数

| 参数名          | 解释                | 类型        | 支持的配置                                                   | 是否必须 | 默认值                       |
| --------------- | ------------------- | ----------- | ------------------------------------------------------------ | -------- | ---------------------------- |
| image_topic     | 订阅的图像topic     | std::string | 根据实际mipi_cam节点配置                                     | 否       | "/image_jpeg"                |
| image_type      | image消息类型       | std::string | "mjpeg"/"mjpeg_shared_mem"<br />"mjpeg"：ros类型jpeg图像<br />"mjpeg_shared_mem"：shared_mem类型jpeg图像 | 否       | "mjpeg"                      |
| only_show_image | 是否只显示图像      | bool        | true/false                                                   | 否       | false                        |
| smart_topic     | 订阅的智能结果topic | std::string | 根据实际算法推理节点配置                                     | 否       | /hobot_mono2d_body_detection |

## 运行

编译成功后，将生成的install路径拷贝到地平线X3开发板上（如果是在X3上编译，忽略拷贝步骤），并执行如下命令运行：

### **Ubuntu**

#### 方式1，ros2 run运行

source setup.bash

~~~shell
export COLCON_CURRENT_PREFIX=./install
source ./install/setup.bash
~~~

运行mipi cam

~~~shell
ros2 run mipi_cam mipi_cam --ros-args -p out_format:=nv12 -p image_width:=960 -p image_height:=544 -p io_method:=shared_mem --log-level error &
~~~

运行hobot_codec进行jpeg编码

~~~shell
ros2 run hobot_codec hobot_codec_republish --ros-args -p channel:=1 -p in_mode:=shared_mem -p in_format:=nv12 -p out_mode:=ros -p out_format:=jpeg -p sub_topic:=/hbmem_img -p pub_topic:=/image_jpeg --ros-args --log-level error &
~~~

启动单目rgb人体、人头、人脸、人手框和人体关键点检测

~~~shell
# 如果是板端编译（无--merge-install编译选项），拷贝命令为cp -r install/PKG_NAME/lib/PKG_NAME/config/ .，其中PKG_NAME为具体的package名。
cp -r install/lib/mono2d_body_detection/config/ .

ros2 run mono2d_body_detection mono2d_body_detection --ros-args --log-level error &
~~~

启动websocket服务

第一次运行要启动webserver服务，运行方法为:

`cd` 到 `install/websocket/lib/websocket/webservice`目录下，然后启动nginx

```plaintext
  chmod +x ./sbin/nginx
  ./sbin/nginx -p .
```

启动websocket节点

~~~shell
ros2 run websocket websocket --ros-args -p image_topic:=/image_jpeg -p image_type:=mjpeg -p smart_topic:=/hobot_mono2d_body_detection
~~~

#### 方式2，launch文件启动

```shell
export COLCON_CURRENT_PREFIX=./install
source ./install/setup.bash

# 已在脚本中启动webserver服务，并切换到mono2d_body_detection目录，不需再拷贝config文件夹到当前目录
ros2 launch websocket hobot_websocket.launch.py
```

### **Linux**

第一次运行要启动webserver服务，运行方法为:

`cd` 到 `install/websocket/lib/websocket/webservice`目录下，然后启动nginx

```plaintext
  chmod +x ./sbin/nginx
  ./sbin/nginx -p .
```

启动各个节点：

```shell
export ROS_LOG_DIR=/userdata/
export LD_LIBRARY_PATH=${LD_LIBRARY_PATH}:./install/lib/

# config中为示例使用的模型，根据实际安装路径进行拷贝
cp -r install/lib/mono2d_body_detection/config/ .

# 启动图片发布pkg
./install/lib/mipi_cam/mipi_cam --ros-args -p out_format:=nv12 -p image_width:=960 -p image_height:=544 -p io_method:=shared_mem --log-level error &
# 启动jpeg图片编码&发布pkg
./install/lib/hobot_codec/hobot_codec_republish --ros-args -p channel:=1 -p in_mode:=shared_mem -p in_format:=nv12 -p out_mode:=ros -p out_format:=jpeg -p sub_topic:=/hbmem_img -p pub_topic:=/image_jpeg --ros-args --log-level error &
# 启动单目rgb人体、人头、人脸、人手框和人体关键点检测pkg
./install/lib/mono2d_body_detection/mono2d_body_detection --ros-args --log-level error &

# 启动web展示pkg
./install/lib/websocket/websocket --ros-args -p image_topic:=/image_jpeg -p image_type:=mjpeg -p smart_topic:=/hobot_mono2d_body_detection &
```

## 注意事项

第一次运行web展示需要启动webserver服务。

# 结果分析

## X3结果展示

```
root@ubuntu:~# ros2 run websocket websocket --ros-args -p image_topic:=/image_jpeg -p image_type:=mjpeg -p smart_topic:=/hobot_mono2d_body_detection
[INFO] [1652694326.097724577] [websocket]: 
Parameter:
 image_topic: /image_jpeg
 image_type: mjpeg
 only_show_image: 0
 smart_topic: /hobot_mono2d_body_detection
[INFO] [1652694326.098510510] [websocket]: Websocket using image jpeg
```

## web效果展示

在PC端输入http://IP 即可查看图像和算法渲染效果（IP为设备IP地址）

# 常见问题
