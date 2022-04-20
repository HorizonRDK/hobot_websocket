

## WEB展示端

### 功能介绍

为了方便预览图像和算法效果，TROS集成了web展示功能，可通过网络将图像和算法结果传输到浏览器端，然后进行渲染显示。

### 准备工作

已安装TROS，并成功运行hobot sensor和算法推理Node。

### 使用介绍

websocket node接收hobot sensor传输的image消息和算法推理输出的智能结果数据，并进行匹配，然后输出渲染。

image消息支持`sensor_msgs::msg::Image`类型消息，`nv12`和`mjpeg`两种格式数据，以及零拷贝的`hbm_img_msgs::msg::HbmMsg1080P`类型消息。

智能结果数据支持`ai_msgs::msg::PerceptionTargets`类型消息，其中`header.stamp`必须和该结果对应的image消息相同，websocket会使用该字段进行消息匹配。

websocket node支持通过parameters设置输入的topic以及图像数据类型和宽高，具体参数如下：

| 参数名       | 解释                                                         | 是否必须 | 默认值                         | 备注 |
| ------------ | ------------------------------------------------------------ | -------- | ------------------------------ | ---- |
| image_topic  | 订阅的图像topic                                              | 否       | "/image_raw"                   |      |
| image_height | 输入图像高                                                   | 否       | 1080                           |      |
| image_width  | 输入图像宽                                                   | 否       | 1920                           |      |
| image_type   | 输入图像类型，可选项为"nv12", "mjpeg", "nv12_hbmem"<br />"nv12"：`sensor_msgs::msg::Image`类型nv12图像<br />"mjpeg"：`sensor_msgs::msg::Image`类型mjpeg图像<br />"nv12_hbmem"：`hbm_img_msgs::msg::HbmMsg1080P`使用零拷贝的nv12图像 | 否       | "nv12"                         |      |
| jpeg_quality | 输入为nv12格式图像时，Jpeg编码质量，mjpeg图像输入可忽略      | 否       | 95                             |      |
| smart_topic  | 订阅的智能结果topic                                          | 否       | "/hobot_mono2d_body_detection" |      |
| smart_height | 智能结果对应的图像高                                         | 否       | 1080                           |      |
| smart_width  | 智能结果对应的图像宽                                         | 否       | 1920                           |      |

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
ros2 run mipi_cam mipi_cam --ros-args -p video_device:=F37 -p out_format:=nv12 -p io_method:=hbmem --log-level warn
```

启动mono2d_body_detection算法


```shell
ros2 run mono2d_body_detection mono2d_body_detection --ros-args -p model_file_name:=/opt/tros/lib/mono2d_body_detection/config/multitask_body_kps_960x544.hbm --log-level warn
```

然后运行websocket节点

```shell
ros2 run websocket websocket --ros-args -p image_topic:=/hbmem_img -p image_type:=nv12_hbmem
```

### 结果分析

在PC端输入http://IP 即可查看图像和算法渲染效果
