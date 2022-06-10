// Copyright (c) 2021 Horizon Robotics.All Rights Reserved.
//
// The material in this file is confidential and contains trade secrets
// of Horizon Robotics Inc. This is proprietary information owned by
// Horizon Robotics Inc. No part of this work may be disclosed,
// reproduced, copied, transmitted, or used in any way for any purpose,
// without the express written permission of Horizon Robotics Inc.

#ifndef WEBSOCKET_INCLUDE_H_
#define WEBSOCKET_INCLUDE_H_

#include <memory>
#include <string>
#include <vector>

#include "ai_msgs/msg/perception_targets.hpp"
#include "hbm_img_msgs/msg/hbm_msg1080_p.hpp"
#include "protobuf/x3.pb.h"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "server/uws_server.h"
#include "threads/threadpool.h"

namespace websocket {

struct compare_frame {
  bool operator()(const sensor_msgs::msg::Image::SharedPtr f1,
                  const sensor_msgs::msg::Image::SharedPtr f2) {
    return ((f1->header.stamp.sec > f2->header.stamp.sec) ||
            ((f1->header.stamp.sec == f2->header.stamp.sec) &&
             (f1->header.stamp.nanosec > f2->header.stamp.nanosec)));
  }
};
struct compare_msg {
  bool operator()(const ai_msgs::msg::PerceptionTargets::SharedPtr m1,
                  const ai_msgs::msg::PerceptionTargets::SharedPtr m2) {
    return ((m1->header.stamp.sec > m2->header.stamp.sec) ||
            ((m1->header.stamp.sec == m2->header.stamp.sec) &&
             (m1->header.stamp.nanosec > m2->header.stamp.nanosec)));
  }
};

class Websocket {
 public:
  Websocket(rclcpp::Node::SharedPtr &nh);
  ~Websocket();

  void OnGetJpegImage(const sensor_msgs::msg::Image::SharedPtr msg);
  void OnGetJpegImageHbmem(const hbm_img_msgs::msg::HbmMsg1080P::SharedPtr msg);
  void OnGetSmartMessage(const ai_msgs::msg::PerceptionTargets::SharedPtr msg);

 private:
  rclcpp::Node::SharedPtr nh_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub_;
  rclcpp::Subscription<ai_msgs::msg::PerceptionTargets>::SharedPtr ai_msg_sub_;
  rclcpp::SubscriptionHbmem<hbm_img_msgs::msg::HbmMsg1080P>::SharedPtr
      image_hbmem_sub_;

  std::shared_ptr<UwsServer> uws_server_;
  std::shared_ptr<std::thread> worker_;
  std::mutex map_smart_mutex_;
  bool map_stop_ = false;
  std::condition_variable map_smart_condition_;

  std::priority_queue<sensor_msgs::msg::Image::SharedPtr,
                      std::vector<sensor_msgs::msg::Image::SharedPtr>,
                      compare_frame>
      x3_frames_;
  std::priority_queue<ai_msgs::msg::PerceptionTargets::SharedPtr,
                      std::vector<ai_msgs::msg::PerceptionTargets::SharedPtr>,
                      compare_msg>
      x3_smart_msg_;

  std::mutex smart_mutex_;
  bool smart_stop_flag_;
  std::mutex video_mutex_;
  bool video_stop_flag_;

  hobot::CThreadPool data_send_thread_;

  std::ofstream output_file_;
  int image_count_ = 0;
  uint64_t frame_id_ = 0;

  std::string image_topic_name_ = "/image_jpeg";
  std::string image_type_ = "mjpeg";
  std::string smart_topic_name_ = "/hobot_mono2d_body_detection";

  bool only_show_image_ = false;
  bool using_hbmem_image_ = false;

  int FrameAddImage(x3::FrameMessage &msg_send,
                    sensor_msgs::msg::Image::SharedPtr frame_msg);
  int FrameAddSmart(x3::FrameMessage &proto_frame_message,
                    ai_msgs::msg::PerceptionTargets::SharedPtr smart_msg);
  int FrameAddSystemInfo(x3::FrameMessage &msg_send);
  int SendImageMessage(sensor_msgs::msg::Image::SharedPtr frame_msg);
  int SendImageSmartMessage(
      ai_msgs::msg::PerceptionTargets::SharedPtr smart_msg,
      sensor_msgs::msg::Image::SharedPtr frame_msg);

  void MessageProcess(void);
};

}  // namespace websocket

#endif