// Copyright (c) 2021 Horizon Robotics.All Rights Reserved.
//
// The material in this file is confidential and contains trade secrets
// of Horizon Robotics Inc. This is proprietary information owned by
// Horizon Robotics Inc. No part of this work may be disclosed,
// reproduced, copied, transmitted, or used in any way for any purpose,
// without the express written permission of Horizon Robotics Inc.

#include "websocket/websocket.h"

namespace websocket {

std::map<int, std::string> gesture_map{{0, ""},
                                       {1, "Finger Heart"},
                                       {2, "Thumb Up"},
                                       {3, "Victory"},
                                       {4, "Mute"},
                                       {5, "Palm"},
                                       {6, "IndexFingerAntiClockwise"},
                                       {7, "IndexFingerClockwise"},
                                       {8, "Pinch"},
                                       {9, "Palmpat"},
                                       {10, "Palm Move"},
                                       {11, "Okay"},
                                       {12, "ThumbLeft"},
                                       {13, "ThumbRight"},
                                       {14, "Awesome"},
                                       {15, "PinchMove"},
                                       {16, "PinchAntiClockwise"},
                                       {17, "PinchClockwise"}};

Websocket::Websocket(rclcpp::Node::SharedPtr &nh) : nh_(nh) {
  uws_server_ = std::make_shared<UwsServer>();
  if (uws_server_->Init()) {
    throw std::runtime_error("Websocket Init uWS server failed");
  }

  if (!worker_) {
    worker_ = std::make_shared<std::thread>(
        std::bind(&Websocket::MessageProcess, this));
  }

  data_send_thread_.CreatThread(1);

  smart_stop_flag_ = false;
  video_stop_flag_ = false;

  // output_file_.open("./out.yuv", std::ios::out | std::ios::binary);

  rcl_interfaces::msg::ParameterDescriptor image_topic_descriptor;
  image_topic_descriptor.description = "image topic name";
  rcl_interfaces::msg::ParameterDescriptor image_type_descriptor;
  image_type_descriptor.description =
      "image type, supported options: mjpeg, mjpeg_shared_mem";
  rcl_interfaces::msg::ParameterDescriptor smart_topic_descriptor;
  smart_topic_descriptor.description = "smart topic name";
  nh_->declare_parameter<std::string>("image_topic", image_topic_name_,
                                      image_topic_descriptor);
  nh_->declare_parameter<std::string>("image_type", image_type_,
                                      image_type_descriptor);
  nh_->declare_parameter<std::string>("smart_topic", smart_topic_name_,
                                      smart_topic_descriptor);

  nh_->declare_parameter<bool>("only_show_image", only_show_image_);

  nh_->get_parameter<std::string>("image_topic", image_topic_name_);
  nh_->get_parameter<std::string>("image_type", image_type_);
  nh_->get_parameter<std::string>("smart_topic", smart_topic_name_);

  nh_->get_parameter<bool>("only_show_image", only_show_image_);

  if (only_show_image_) {
    RCLCPP_INFO_STREAM(nh_->get_logger(),
                       "\nParameter:"
                           << "\n image_topic: " << image_topic_name_
                           << "\n image_type: " << image_type_
                           << "\n only_show_image: " << only_show_image_);
  } else {
    RCLCPP_INFO_STREAM(nh_->get_logger(),
                       "\nParameter:"
                           << "\n image_topic: " << image_topic_name_
                           << "\n image_type: " << image_type_
                           << "\n only_show_image: " << only_show_image_
                           << "\n smart_topic: " << smart_topic_name_);
  }

  if (image_type_ == "mjpeg") {
    RCLCPP_INFO(nh_->get_logger(), "Websocket using image jpeg");
    using_hbmem_image_ = false;
    image_sub_ = nh_->create_subscription<sensor_msgs::msg::Image>(
        image_topic_name_, 10,
        std::bind(&Websocket::OnGetJpegImage, this, std::placeholders::_1));
  } else if (image_type_ == "mjpeg_shared_mem") {
    RCLCPP_INFO(nh_->get_logger(), "Websocket using image jpeg shared memory");
    using_hbmem_image_ = true;
    image_hbmem_sub_ =
        nh_->create_subscription_hbmem<hbm_img_msgs::msg::HbmMsg1080P>(
            image_topic_name_, 10,
            std::bind(&Websocket::OnGetJpegImageHbmem, this,
                      std::placeholders::_1));
  } else {
    RCLCPP_ERROR(nh_->get_logger(), "Websocket unsupported image type");
    throw std::runtime_error("Websocket unsupported image type");
  }

  if (!only_show_image_) {
    ai_msg_sub_ = nh_->create_subscription<ai_msgs::msg::PerceptionTargets>(
        smart_topic_name_, 10,
        std::bind(&Websocket::OnGetSmartMessage, this, std::placeholders::_1));
  }
}

Websocket::~Websocket() {
  {
    std::lock_guard<std::mutex> smart_lock(smart_mutex_);
    smart_stop_flag_ = true;
  }
  {
    std::lock_guard<std::mutex> video_lock(video_mutex_);
    video_stop_flag_ = true;
  }
  {
    if (worker_ && worker_->joinable()) {
      map_stop_ = true;
      map_smart_condition_.notify_one();
      worker_->join();
      worker_ = nullptr;
      RCLCPP_INFO(nh_->get_logger(), "Websocket stop worker");
    }
  }

  uws_server_->DeInit();
}

void Websocket::OnGetJpegImage(const sensor_msgs::msg::Image::SharedPtr msg) {
  {
    std::lock_guard<std::mutex> video_lock(video_mutex_);
    if (video_stop_flag_) {
      RCLCPP_WARN(nh_->get_logger(),
                  "Aleardy stop, Websocket Feedvideo return");
      return;
    }
  }

  {
    std::lock_guard<std::mutex> lock(map_smart_mutex_);
    x3_frames_.push(msg);
  }
  map_smart_condition_.notify_one();
}

void Websocket::OnGetJpegImageHbmem(
    const hbm_img_msgs::msg::HbmMsg1080P::SharedPtr msg) {
  {
    std::lock_guard<std::mutex> video_lock(video_mutex_);
    if (video_stop_flag_) {
      RCLCPP_WARN(nh_->get_logger(),
                  "Aleardy stop, Websocket Feedvideo return");
      return;
    }
  }

  auto image = std::make_shared<sensor_msgs::msg::Image>();
  image->header.stamp = msg->time_stamp;
  image->header.frame_id = "default_cam";

  image->width = msg->width;
  image->height = msg->height;
  image->step = msg->width;
  image->encoding = "jpeg";
  image->data.resize(msg->data_size);
  memcpy(image->data.data(), msg->data.data(), msg->data_size);

  {
    std::lock_guard<std::mutex> lock(map_smart_mutex_);
    x3_frames_.push(image);
  }
  map_smart_condition_.notify_one();
}

void Websocket::OnGetSmartMessage(
    const ai_msgs::msg::PerceptionTargets::SharedPtr msg) {
  {
    std::lock_guard<std::mutex> smart_lock(smart_mutex_);
    if (smart_stop_flag_) {
      RCLCPP_WARN(nh_->get_logger(),
                  "Aleardy stop, Websocket FeedSmart return");
      return;
    }
  }

  {
    std::lock_guard<std::mutex> smart_lock(map_smart_mutex_);
    x3_smart_msg_.push(msg);
  }
  map_smart_condition_.notify_one();
}

int Websocket::FrameAddSmart(
    x3::FrameMessage &proto_frame_message,
    ai_msgs::msg::PerceptionTargets::SharedPtr smart_msg) {
  proto_frame_message.set_timestamp_(smart_msg->header.stamp.sec * 1000000000 +
                                     smart_msg->header.stamp.nanosec);
  proto_frame_message.set_sequence_id_(frame_id_++);

  auto static_msg = proto_frame_message.mutable_statistics_msg_();
  auto fps_attrs = static_msg->add_attributes_();
  fps_attrs->set_type_("fps");
  fps_attrs->set_value_(smart_msg->fps);
  fps_attrs->set_value_string_(std::to_string(smart_msg->fps));

  auto smart = proto_frame_message.mutable_smart_msg_();
  smart->set_timestamp_(smart_msg->header.stamp.sec * 1000000000 +
                        smart_msg->header.stamp.nanosec);
  smart->set_error_code_(0);
  for (auto smart_target : smart_msg->targets) {
    auto target = smart->add_targets_();
    target->set_track_id_(smart_target.track_id);
    target->set_type_(smart_target.type);

    if (smart_target.rois.size() == 1 &&
    !smart_target.rois.front().type.empty()) {
      target->set_type_(smart_target.rois.front().type);
    }

    // rois
    for (auto smart_roi : smart_target.rois) {
      auto proto_box = target->add_boxes_();
      proto_box->set_type_(smart_roi.type);
      auto point1 = proto_box->mutable_top_left_();
      point1->set_x_(smart_roi.rect.x_offset);
      point1->set_y_(smart_roi.rect.y_offset);
      // point1->set_score_(1.0);
      auto point2 = proto_box->mutable_bottom_right_();
      point2->set_x_(smart_roi.rect.x_offset + smart_roi.rect.width);
      point2->set_y_(smart_roi.rect.y_offset + smart_roi.rect.height);
      // point2->set_score_(1.0);
      // proto_box->set_score(1.0);
    }

    // attributes
    for (auto smart_attributes : smart_target.attributes) {
      auto attrs = target->add_attributes_();
      attrs->set_type_(smart_attributes.type);
      attrs->set_value_(smart_attributes.value);
      // attrs->set_score_(1.0);
      if (smart_attributes.type == "gesture") {
        auto gesture_string = gesture_map.find(smart_attributes.value);
        if (gesture_string != gesture_map.end()) {
          attrs->set_value_string_(gesture_string->second);
        } else {
          attrs->set_value_string_(std::to_string(smart_attributes.value));
        }
      } else if (smart_attributes.type == "gender") {
        if (smart_attributes.value == 1) {
          attrs->set_value_string_("男");
        } else if (smart_attributes.value == -1) {
          attrs->set_value_string_("女");
        } else {
          attrs->set_value_string_(std::to_string(smart_attributes.value));
        }
      } else {
        attrs->set_value_string_(std::to_string(smart_attributes.value));
      }
    }

    // points
    for (auto smart_points : smart_target.points) {
      auto proto_points = target->add_points_();
      std::string pt_type = "body_kps";
      if (smart_points.type == "body_kps") {
        pt_type = "body_landmarks";
      } else if (smart_points.type == "hand_kps") {
        pt_type = "hand_landmarks";
      } else if (smart_points.type == "face_kps") {
        pt_type = "lmk_106pts";
      }
      proto_points->set_type_(pt_type);
      for (auto smart_point : smart_points.point) {
        auto point = proto_points->add_points_();
        point->set_x_(smart_point.x);
        point->set_y_(smart_point.y);
        point->set_score_(1.0);
      }
    }

    // captures
    // for (auto smart_captures : smart_target.captures) {
    //     auto captures_ = target->add_captures_();
    // }
  }

  return 0;
}

int Websocket::FrameAddImage(x3::FrameMessage &msg_send,
                             sensor_msgs::msg::Image::SharedPtr frame_msg) {
  msg_send.set_timestamp_(frame_msg->header.stamp.sec * 1000000000 +
                          frame_msg->header.stamp.nanosec);
  auto image = msg_send.mutable_img_();
  image->set_buf_((const char *)frame_msg->data.data(), frame_msg->data.size());
  image->set_type_("jpeg");
  image->set_width_(frame_msg->width);
  image->set_height_(frame_msg->height);
  return 0;
}

int Websocket::FrameAddSystemInfo(x3::FrameMessage &msg_send) {
  std::string cpu_rate_file =
      "/sys/devices/system/cpu/cpufreq/policy0/cpuinfo_cur_freq";
  std::string temp_file = "/sys/class/thermal/thermal_zone0/temp";
  std::ifstream ifs(cpu_rate_file.c_str());
  if (!ifs.is_open()) {
    RCLCPP_ERROR(nh_->get_logger(), "open config file %s failed",
                 cpu_rate_file.c_str());
    return -1;
  }
  std::stringstream ss;
  std::string str;
  ss << ifs.rdbuf();
  ss >> str;
  ifs.close();
  auto Statistics_msg_ = msg_send.mutable_statistics_msg_();
  auto attrs = Statistics_msg_->add_attributes_();
  attrs->set_type_("cpu");
  attrs->set_value_string_(str.c_str());

  ifs.clear();
  ss.clear();
  ifs.open(temp_file.c_str());
  ss << ifs.rdbuf();
  ss >> str;

  auto temp_attrs = Statistics_msg_->add_attributes_();
  temp_attrs->set_type_("temp");
  temp_attrs->set_value_string_(str.c_str());
  return 0;
}

int Websocket::SendImageMessage(sensor_msgs::msg::Image::SharedPtr frame_msg) {
  x3::FrameMessage msg_send;
  FrameAddImage(msg_send, frame_msg);
  FrameAddSystemInfo(msg_send);
  std::string proto_send;
  msg_send.SerializeToString(&proto_send);
  uws_server_->Send(proto_send);
  return 0;
}

int Websocket::SendImageSmartMessage(
    ai_msgs::msg::PerceptionTargets::SharedPtr smart_msg,
    sensor_msgs::msg::Image::SharedPtr frame_msg) {
  x3::FrameMessage msg_send;
  FrameAddImage(msg_send, frame_msg);
  FrameAddSmart(msg_send, smart_msg);
  FrameAddSystemInfo(msg_send);
  std::string proto_send;
  msg_send.SerializeToString(&proto_send);
  uws_server_->Send(proto_send);
  return 0;
}

void Websocket::MessageProcess() {
  while (!map_stop_) {
    std::unique_lock<std::mutex> lock(map_smart_mutex_);
    map_smart_condition_.wait(lock);
    if (map_stop_) {
      break;
    }
    if (only_show_image_) {
      while (!x3_frames_.empty()) {
        auto frame = x3_frames_.top();
        int task_num = data_send_thread_.GetTaskNum();
        if (task_num < 3) {
          data_send_thread_.PostTask(
              std::bind(&Websocket::SendImageMessage, this, frame));
        }
        x3_frames_.pop();
      }
    } else {
      while (!x3_smart_msg_.empty() && !x3_frames_.empty()) {
        auto msg = x3_smart_msg_.top();
        auto frame = x3_frames_.top();
        if (msg->header.stamp == frame->header.stamp) {
          int task_num = data_send_thread_.GetTaskNum();
          if (task_num < 3) {
            data_send_thread_.PostTask(
                std::bind(&Websocket::SendImageSmartMessage, this, msg, frame));
          }
          x3_smart_msg_.pop();
          x3_frames_.pop();
        } else if ((msg->header.stamp.sec * 1000000000 +
                    msg->header.stamp.nanosec) >
                   (frame->header.stamp.sec * 1000000000 +
                    frame->header.stamp.nanosec)) {
          x3_frames_.pop();
        } else {
          x3_smart_msg_.pop();
        }
      }
    }

    if (x3_smart_msg_.size() > 20) {
      RCLCPP_WARN(nh_->get_logger(),
                  "web socket has cache smart message nun > 20");
    }
    if (x3_frames_.size() > 20) {
      RCLCPP_WARN(nh_->get_logger(), "web socket has cache image nun > 20");
    }
  }
}

}  // namespace websocket
