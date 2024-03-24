English| [简体中文](./README_cn.md)

# WEB Display End

## Function Introduction

To facilitate the preview of images and algorithm effects, TogetherROS integrates web display function, transferring images and algorithm results to the browser end through the network for rendering and displaying.

## Compilation

### Dependencies

ROS packages:

- hbm_img_msgs
- ai_msgs

hbm_img_msgs is a custom message format used to publish shared memory type image data, defined in hobot_msgs.

ai_msgs is a custom message format used to publish algorithm inference results, defined in hobot_msgs.

### Development Environment

- Programming Language: C/C++
- Development Platform: X3/X86
- System Version: Ubuntu 20.04
- Compilation Toolchain: Linux GCC 9.3.0/Linaro GCC 9.3.0

### Compilation

Support compilation on X3/X86 Ubuntu system and cross-compilation of x3 executable on x86 Ubuntu using Docker.

#### Compilation on X3/X86 Ubuntu Platform

1. Compilation Environment Confirmation

   - Ubuntu system is Ubuntu 20.04.
   - The current compilation terminal has set the TogetherROS environment variable: `source PATH/setup.bash`. Here, PATH is the installation path of TogetherROS.
   - ROS2 compilation tool colcon is installed, installation command: `pip install -U colcon-common-extensions`

2. Compilation

Compilation command: `colcon build --merge-install --packages-select websocket`

#### Cross-compilation on x86 Ubuntu Docker

1. Compilation Environment Confirmation

   - Compilation in Docker with TogetherROS already installed. For Docker installation, cross-compilation instructions, and TogetherROS compilation and deployment details, refer to the README.md in the robot development platform robot_dev_config repository.- Compilation command:

   ```shell
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

### Notes

The TogetherROS installation package already includes the websocket package, which users can directly use without the need for separate compilation. If users develop new features based on the source code, separate compilation and validation are required.

## User Guide

websocket supports running on X3/X86 Ubuntu 20.04 systems and x3 yocto Linux systems.

### Dependencies

websocket receives image messages and smart result messages, matches them based on timestamps, and then outputs them for rendering on the web client, or displays images individually.

Image messages support `sensor_msgs::msg::CompressedImage` and `shared_mem` of type `hbm_img_msgs::msg::HbmMsg1080P`, which must be MJPEG encoded image data.

Smart result messages support messages of type `ai_msgs::msg::PerceptionTargets`, where `header.stamp` must match the timestamp of the corresponding image message. Websocket will use this field for message matching, and the width and height of the smart result must be consistent with the resolution of the received image.

Specific package dependencies include:

- mipi_cam: Starts the mipi cam and publishes nv12 type image messages
- hobot_image_publisher: Inputs images or videos and publishes nv12 type image messages
- hobot_usb_cam: Gets images from a USB camera and publishes images in MJPEG format
- hobot_codec: Encodes nv12 images published by mipi_cam into JPEG format required by websocket
- mono2d_body_detection: Receives nv12 format data, performs algorithm inference, and publishes perception messages for human body, head, face, and hand bounding boxes

### Parameters

| Parameter Name  | Description          | Type        | Supported Configurations                                      | Required | Default Value                 |
| --------------- | ------------------- | ----------- | ------------------------------------------------------------ | -------- | ---------------------------- |
| image_topic     | Subscribed image topic     | std::string | Depending on the actual configuration of the mipi_cam node  | No       | "/image_jpeg"                |
| image_type      | Image message type       | std::string | "mjpeg"/"mjpeg_shared_mem"<br />"mjpeg": ros type jpeg image<br />"mjpeg_shared_mem": shared_mem type jpeg image | No       | "mjpeg"                      |
| only_show_image | Display image only      | bool        | true/false                                                   | No       | false                        |
| smart_topic     | Subscribed smart result topic | std::string | Depending on the actual configuration of the algorithm inference node | No       | /hobot_mono2d_body_detection |
| output_fps     | Output images according to specified frame rate | int | [1, 30], configurations beyond this range mean no frame rate control | No       | 0 (no frame rate control) |

### RunningAfter successful compilation, if it is Docker cross compilation, the generated install path needs to be copied to Horizon X3 development board, other methods do not require this. The running steps are as follows:

#### **x86 Ubuntu system**

source setup.bash

~~~shell
source ./install/setup.bash
~~~

Run hobot_usb_cam to publish MJPEG images

~~~shell
ros2 run hobot_usb_cam hobot_usb_cam --ros-args -p pixel_format:=mjpeg -p image_width:=1280 -p image_height:=720 -p zero_copy:=false -p video_device:="/dev/video0" --log-level error &
~~~

Start the websocket service

For the first time, you need to start the webserver service by running:

```shell
ros2 launch websocket websocket_service.launch.py
```

Start the websocket node

~~~shell
ros2 run websocket websocket --ros-args -p image_topic:=/image -p image_type:=mjpeg -p only_show_image:=true
~~~

#### **x3 Ubuntu**

##### Method 1, running with ros2 run

source setup.bash

~~~shell
source ./install/setup.bash
~~~

Run mipi cam

~~~shell
ros2 run mipi_cam mipi_cam --ros-args -p out_format:=nv12 -p image_width:=960 -p image_height:=544 -p io_method:=shared_mem --log-level error &
~~~

Run hobot_codec for JPEG encoding

~~~shell```shell
ros2 run hobot_codec hobot_codec_republish --ros-args -p channel:=1 -p in_mode:=shared_mem -p in_format:=nv12 -p out_mode:=ros -p out_format:=jpeg -p sub_topic:=/hbmem_img -p pub_topic:=/image_jpeg --ros-args --log-level error &
```

Start single-camera RGB human body, head, face, hand bounding box detection

```shell
# If compiling on the board end (without the --merge-install compilation option), the copy command is cp -r install/PKG_NAME/lib/PKG_NAME/config/ ., where PKG_NAME is the specific package name.
cp -r install/lib/mono2d_body_detection/config/ .

ros2 run mono2d_body_detection mono2d_body_detection --ros-args --log-level error &
```

Start websocket service

To start the webserver service for the first time, run the following command:

```shell
ros2 launch websocket websocket_service.launch.py
```

Start websocket node

```shell
ros2 run websocket websocket --ros-args -p image_topic:=/image_jpeg -p image_type:=mjpeg -p smart_topic:=/hobot_mono2d_body_detection
```

##### Method 2, launch file start

```shell
source ./install/setup.bash

ros2 launch mipi_cam mipi_cam.launch.py mipi_video_device:=F37
```

```shell
source ./install/setup.bash

ros2 launch hobot_codec hobot_codec_encode.launch.py
```

```shell
source ./install/setup.bash

# Webserver service already started in the script
ros2 launch websocket websocket.launch.py websocket_image_topic:=/image_jpeg websocket_only_show_image:=true
```

#### **x3 Linux**

To start the webserver service for the first time, run the following command:`cd` to the directory `install/websocket/lib/websocket/webservice`, and then start nginx

```shell
  ./sbin/nginx -p .
```

Start each node:

```shell
export ROS_LOG_DIR=/userdata/
export LD_LIBRARY_PATH=${LD_LIBRARY_PATH}:./install/lib/

# Copy the example model in the config folder based on the actual installation path
cp -r install/lib/mono2d_body_detection/config/ .

# Start image publishing package
./install/lib/mipi_cam/mipi_cam --ros-args -p out_format:=nv12 -p image_width:=960 -p image_height:=544 -p io_method:=shared_mem --log-level error &
# Start jpeg image encoding & publishing package
./install/lib/hobot_codec/hobot_codec_republish --ros-args -p channel:=1 -p in_mode:=shared_mem -p in_format:=nv12 -p out_mode:=ros -p out_format:=jpeg -p sub_topic:=/hbmem_img -p pub_topic:=/image_jpeg --ros-args --log-level error &
# Start mono2d body detection for single RGB image nodes including human body, head, face, hand box, and key points detection package
./install/lib/mono2d_body_detection/mono2d_body_detection --ros-args --log-level error &

# Start web display package
./install/lib/websocket/websocket --ros-args -p image_topic:=/image_jpeg -p image_type:=mjpeg -p smart_topic:=/hobot_mono2d_body_detection &
```

### Notes

To run the web display for the first time, the webserver service needs to be started.

## Results Analysis

### Result LOG Display

```text
root@ubuntu:~# ros2 run websocket websocket --ros-args -p image_topic:=/image_jpeg -p image_type:=mjpeg -p smart_topic:=/hobot_mono2d_body_detection
[INFO] [1652694326.097724577] [websocket]: 
Parameter:
 image_topic: /image_jpeg
 image_type: mjpeg
 only_show_image: 0
 smart_topic: /hobot_mono2d_body_detection
[INFO] [1652694326.098510510] [websocket]: Websocket using image jpeg
```

### Web Display Effect

Use Google Chrome or Edge, enter <http://IP:8000> to view the image and algorithm rendering effect (IP is the device's IP address).## Frequently Asked Questions

### Failed to start webserver

The webserver service needs to use port 8000. If the port is already in use, the startup will fail.

You can use the command `lsof -i:8000` to check which process is occupying port 8000. Use `kill <PID>` to terminate the process using port 8000, and then restart the server.

If the user does not want to stop the current service occupying port 8000, they can modify the `listen` port number in the **webservice/conf/nginx.conf** configuration file to a port number greater than 1024 and not in use. Note that after modifying this port number, the URL used on the client side also needs to be updated accordingly.