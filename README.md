

## WEB展示端

### 功能介绍

为了方便预览图像和算法效果，TROS集成了web展示功能，可通过网络将图像和算法结果传输到浏览器端，然后进行渲染显示。

### 准备工作

已安装TROS，并成功运行hobot sensor和算法推理Node。

### 使用介绍

websocket node接收hobot sensor传输的image消息和算法推理输出的智能结果数据，并进行匹配，然后输出渲染，同时也可单独显示视频。

image消息支持`sensor_msgs::msg::Image`以及零拷贝的`hbm_img_msgs::msg::HbmMsg1080P`类型消息，必须为hobot codec输出的jpeg格式数据。

智能结果数据支持`ai_msgs::msg::PerceptionTargets`类型消息，其中`header.stamp`必须和该结果对应的image消息相同，websocket会使用该字段进行消息匹配。

websocket node支持通过parameters设置输入的topic以及图像数据类型和宽高，具体参数如下：

| 参数名          | 解释                                                         | 是否必须 | 默认值                         | 备注 |
| --------------- | ------------------------------------------------------------ | -------- | ------------------------------ | ---- |
| image_topic     | 订阅的图像topic                                              | 否       | "/image_jpeg"                  |      |
| image_type      | image消息类型，可选项为"mjpeg", "mjpeg_shared_mem"<br />"mjpeg"：`sensor_msgs::msg::Image`类型jpeg图像<br />"mjpeg_shared_mem"：`hbm_img_msgs::msg::HbmMsg1080P`使用零拷贝的jpeg图像 | 否       | "mjpeg"                         |      |
| only_show_image | 是否只显示视频                                               | 否       | false                          |      |
| smart_topic     | 订阅的智能结果topic                                          | 否       | "/hobot_mono2d_body_detection" |      |

#### 运行

第一次运行要启动webserver服务，运行方法为:

`cd` 到 `install/websocket/lib/websocket//webservice`目录下，然后启动nginx

```
  chmod +x ./sbin/nginx
  ./sbin/nginx -p .
```

启动sensor和算法推理节点，这个根据实际使用的sensor和运行的算法而定，下面以sensor为f37和运行mono2d_body_detection算法为例。

启动hobot sensor，输出hbmem的nv12类型图像

```shell
ros2 run mipi_cam mipi_cam --ros-args -p video_device:=F37 -p out_format:=nv12 -p io_method:=shared_mem --log-level warn
```

启动mono2d_body_detection算法


```shell
ros2 run mono2d_body_detection mono2d_body_detection --ros-args -p model_file_name:=/opt/tros/lib/mono2d_body_detection/config/multitask_body_kps_960x544.hbm --log-level warn
```

运行hobot codec节点

~~~shell
ros2 run hobot_codec hobot_codec_republish --ros-args -p channel:=0 -p in_mode:=shared_mem -p in_format:=nv12 -p out_mode:=ros -p out_format:=jpeg -p sub_topic:=/hbmem_img -p pub_topic:=/image_jpeg
~~~



然后运行websocket节点

```shell
ros2 run websocket websocket
```

### 结果分析

在PC端输入http://IP 即可查看图像和算法渲染效果（IP为设备IP地址）
