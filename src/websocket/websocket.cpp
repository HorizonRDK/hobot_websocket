// Copyright (c) 2021 Horizon Robotics.All Rights Reserved.
//
// The material in this file is confidential and contains trade secrets
// of Horizon Robotics Inc. This is proprietary information owned by
// Horizon Robotics Inc. No part of this work may be disclosed,
// reproduced, copied, transmitted, or used in any way for any purpose,
// without the express written permission of Horizon Robotics Inc.

#include "websocket/websocket.h"

namespace websocket {

Websocket::Websocket(rclcpp::Node::SharedPtr &nh) : nh_(nh) {
  uws_server_ = std::make_shared<UwsServer>();
  if (uws_server_->Init()) {
    throw std::runtime_error("Websocket Init uWS server failed");
  }

  if (!worker_) {
    worker_ = std::make_shared<std::thread>(
        std::bind(&Websocket::MapSmartProc, this));
  }

  data_send_thread_.CreatThread(1);

  smart_stop_flag_ = false;
  video_stop_flag_ = false;

  // output_file_.open("./out.yuv", std::ios::out | std::ios::binary);

  rcl_interfaces::msg::ParameterDescriptor image_topic_descriptor;
  image_topic_descriptor.description = "image topic name";
  rcl_interfaces::msg::ParameterDescriptor image_type_descriptor;
  image_type_descriptor.description =
      "image type, supported options: nv12, mjpeg, nv12_hbmem";
  rcl_interfaces::msg::ParameterDescriptor smart_topic_descriptor;
  smart_topic_descriptor.description = "smart topic name";
  nh_->declare_parameter<std::string>("image_topic", image_topic_name_,
                                      image_topic_descriptor);
  nh_->declare_parameter<std::string>("image_type", image_type_,
                                      image_type_descriptor);
  nh_->declare_parameter<std::string>("smart_topic", smart_topic_name_,
                                      smart_topic_descriptor);
  nh_->declare_parameter<int>("image_width", image_width_);
  nh_->declare_parameter<int>("image_height", image_height_);
  nh_->declare_parameter<int>("jpeg_quality", jpeg_quality_);
  nh_->declare_parameter<int>("smart_width", smart_width_);
  nh_->declare_parameter<int>("smart_height", smart_height_);

  nh_->get_parameter<std::string>("image_topic", image_topic_name_);
  nh_->get_parameter<std::string>("image_type", image_type_);
  nh_->get_parameter<std::string>("smart_topic", smart_topic_name_);
  nh_->get_parameter<int>("image_width", image_width_);
  nh_->get_parameter<int>("image_height", image_height_);
  nh_->get_parameter<int>("jpeg_quality", jpeg_quality_);
  nh_->get_parameter<int>("smart_width", smart_width_);
  nh_->get_parameter<int>("smart_height", smart_height_);

  RCLCPP_INFO_STREAM(nh_->get_logger(),
                     "Parameter:"
                         << "\n image_topic: " << image_topic_name_
                         << "\n image_type: " << image_type_
                         << "\n image_width: " << image_width_
                         << "\n image_height: " << image_height_
                         << "\n jpeg_quality: " << jpeg_quality_
                         << "\n smart_topic: " << smart_topic_name_
                         << "\n smart_width: " << smart_width_
                         << "\n smart_height: " << smart_height_);

  if (image_type_ == "nv12") {
    RCLCPP_INFO(nh_->get_logger(), "Websocket using image nv12");
    if (MediaCodecManagerInit()) {
      throw std::runtime_error("Websocket Init Media Codec Manager failed");
    }
    image_sub_ = nh_->create_subscription<sensor_msgs::msg::Image>(
        image_topic_name_, 10,
        std::bind(&Websocket::OnGetYUVImage, this, std::placeholders::_1));
  } else if (image_type_ == "mjpeg") {
    RCLCPP_INFO(nh_->get_logger(), "Websocket using image mjpeg");
    image_sub_ = nh_->create_subscription<sensor_msgs::msg::Image>(
        image_topic_name_, 10,
        std::bind(&Websocket::OnGetJpegImage, this, std::placeholders::_1));
  } else if (image_type_ == "nv12_hbmem") {
    RCLCPP_INFO(nh_->get_logger(), "Websocket using image nv12 hbmem");
    if (MediaCodecManagerInit()) {
      throw std::runtime_error("Websocket Init Media Codec Manager failed");
    }
    image_hbmem_sub_ =
        nh_->create_subscription_hbmem<hbm_img_msgs::msg::HbmMsg1080P>(
            image_topic_name_, 10,
            std::bind(&Websocket::OnGetYUVImageHbmem, this,
                      std::placeholders::_1));
  } else {
    RCLCPP_ERROR(nh_->get_logger(), "Websocket unsupported image type");
    throw std::runtime_error("Websocket unsupported image type");
  }

  ai_msg_sub_ = nh_->create_subscription<ai_msgs::msg::PerceptionTargets>(
      smart_topic_name_, 10,
      std::bind(&Websocket::OnGetSmartMessage, this, std::placeholders::_1));
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

  MediaCodecManagerDeInit();
  uws_server_->DeInit();
}

int Websocket::MediaCodecManagerInit() {
  auto &manager = horizon::vision::MediaCodecManager::Get();
  auto rv = manager.ModuleInit();
  if (rv) return rv;
  venc_chn_ = manager.GetEncodeChn();
  rv = manager.EncodeChnInit(venc_chn_, PT_JPEG, image_width_, image_height_, 8,
                             HB_PIXEL_FORMAT_NV12, 0, 6000);
  if (rv) return rv;
  rv = manager.SetUserQfactorParams(venc_chn_, jpeg_quality_);
  if (rv) return rv;
  rv = manager.EncodeChnStart(venc_chn_);
  if (rv) return rv;
  rv = manager.VbBufInit(venc_chn_, image_width_, image_height_, image_width_,
                         image_width_ * image_height_ * 3 / 2, 8, 1);
  return rv;
}

int Websocket::MediaCodecManagerDeInit() {
  auto &manager = horizon::vision::MediaCodecManager::Get();
  manager.EncodeChnStop(venc_chn_);
  manager.EncodeChnDeInit(venc_chn_);
  manager.VbBufDeInit(venc_chn_);
  manager.ModuleDeInit();
  return 0;
}

void Websocket::OnGetYUVImage(const sensor_msgs::msg::Image::SharedPtr msg) {
  {
    std::lock_guard<std::mutex> video_lock(video_mutex_);
    if (video_stop_flag_) {
      RCLCPP_WARN(nh_->get_logger(),
                  "Aleardy stop, Websocket Feedvideo return");
      return;
    }
  }
  // auto time_now0 = std::chrono::duration_cast<std::chrono::microseconds>(
  //                      std::chrono::steady_clock::now().time_since_epoch())
  //                      .count();
  if (EncodeJpeg(msg)) {
    RCLCPP_WARN(nh_->get_logger(), "Websocket EncodeJpeg failed");
    return;
  }
  // auto time_now1 = std::chrono::duration_cast<std::chrono::microseconds>(
  //                      std::chrono::steady_clock::now().time_since_epoch())
  //                      .count();

  // RCLCPP_INFO(
  //     nh_->get_logger(),
  //     "Websocket OnGetYUVImage %d %d %s, (%d, %d), time cost %dus, size %d",
  //     msg->width, msg->height, msg->encoding.c_str(), msg->header.stamp.sec,
  //     msg->header.stamp.nanosec, time_now1 - time_now0, msg->data.size());
  {
    std::lock_guard<std::mutex> lock(map_smart_mutex_);
    x3_frames_.push(msg);
  }
  map_smart_condition_.notify_one();
}

void Websocket::OnGetYUVImageHbmem(
    const hbm_img_msgs::msg::HbmMsg1080P::SharedPtr msg) {
  {
    std::lock_guard<std::mutex> video_lock(video_mutex_);
    if (video_stop_flag_) {
      RCLCPP_WARN(nh_->get_logger(),
                  "Aleardy stop, Websocket Feedvideo return");
      return;
    }
  }
  // auto time_now0 = std::chrono::duration_cast<std::chrono::microseconds>(
  //                      std::chrono::steady_clock::now().time_since_epoch())
  //                      .count();
  auto jpeg_image = std::make_shared<sensor_msgs::msg::Image>();
  if (EncodeJpeg(msg, jpeg_image)) {
    RCLCPP_WARN(nh_->get_logger(), "Websocket EncodeJpeg failed");
    return;
  }
  // auto time_now1 = std::chrono::duration_cast<std::chrono::microseconds>(
  //                      std::chrono::steady_clock::now().time_since_epoch())
  //                      .count();

  // RCLCPP_INFO(
  //     nh_->get_logger(),
  //     "Websocket OnGetYUVImage %d %d %s, (%d, %d), time cost %dus, size %d",
  //     msg->width, msg->height, msg->encoding.c_str(), msg->header.stamp.sec,
  //     msg->header.stamp.nanosec, time_now1 - time_now0, msg->data.size());
  {
    std::lock_guard<std::mutex> lock(map_smart_mutex_);
    x3_frames_.push(jpeg_image);
  }
  map_smart_condition_.notify_one();
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

int Websocket::EncodeJpeg(const sensor_msgs::msg::Image::SharedPtr msg) {
  iot_venc_src_buf_t *frame_buf = nullptr;
  iot_venc_stream_buf_t *stream_buf = nullptr;

  auto &manager = horizon::vision::MediaCodecManager::Get();
  auto rv = manager.GetVbBuf(venc_chn_, &frame_buf);
  // copy data to frame buf
  auto img_y_size = msg->width * msg->height;
  auto img_uv_size = img_y_size / 2;

  auto y_vaddr = msg->data.data();
  auto c_vaddr = y_vaddr + img_y_size;

  // if (image_count_++ == 100) {
  //   output_file_.write((char *)y_vaddr, img_y_size + img_uv_size);
  // }

  memcpy(frame_buf->frame_info.vir_ptr[0], reinterpret_cast<uint8_t *>(y_vaddr),
         img_y_size);
  memcpy(frame_buf->frame_info.vir_ptr[1], reinterpret_cast<uint8_t *>(c_vaddr),
         img_uv_size);

  rv = manager.EncodeYuvToJpg(venc_chn_, frame_buf, &stream_buf);
  if (rv) return rv;

  auto data_ptr = stream_buf->stream_info.pstPack.vir_ptr;
  auto data_size = stream_buf->stream_info.pstPack.size;

  msg->data.resize(data_size);
  memcpy(msg->data.data(), data_ptr, data_size);

  msg->encoding = "jpeg";
  rv = manager.FreeStream(venc_chn_, stream_buf);
  if (rv) return rv;

  rv = manager.FreeVbBuf(venc_chn_, frame_buf);

  return rv;
}

int Websocket::EncodeJpeg(const hbm_img_msgs::msg::HbmMsg1080P::SharedPtr msg,
                          sensor_msgs::msg::Image::SharedPtr image_jpeg) {
  iot_venc_src_buf_t *frame_buf = nullptr;
  iot_venc_stream_buf_t *stream_buf = nullptr;

  auto &manager = horizon::vision::MediaCodecManager::Get();
  auto rv = manager.GetVbBuf(venc_chn_, &frame_buf);
  // copy data to frame buf
  auto img_y_size = msg->width * msg->height;
  auto img_uv_size = img_y_size / 2;

  auto y_vaddr = msg->data.data();
  auto c_vaddr = y_vaddr + img_y_size;

  // if (image_count_++ == 100) {
  //   output_file_.write((char *)y_vaddr, img_y_size + img_uv_size);
  // }

  memcpy(frame_buf->frame_info.vir_ptr[0], reinterpret_cast<uint8_t *>(y_vaddr),
         img_y_size);
  memcpy(frame_buf->frame_info.vir_ptr[1], reinterpret_cast<uint8_t *>(c_vaddr),
         img_uv_size);

  rv = manager.EncodeYuvToJpg(venc_chn_, frame_buf, &stream_buf);
  if (rv) return rv;

  auto data_ptr = stream_buf->stream_info.pstPack.vir_ptr;
  auto data_size = stream_buf->stream_info.pstPack.size;

  // header:
  //   stamp:
  //     sec: 1649992845
  //     nanosec: 426774678
  //   frame_id: default_cam
  // height: 1080
  // width: 1920
  // encoding: nv12
  // is_bigendian: 0
  // step: 1920
  image_jpeg->header.stamp = msg->time_stamp;
  image_jpeg->header.frame_id = "default_cam";

  image_jpeg->width = msg->width;
  image_jpeg->height = msg->height;
  image_jpeg->step = msg->width;
  image_jpeg->encoding = "jpeg";

  image_jpeg->data.resize(data_size);
  memcpy(image_jpeg->data.data(), data_ptr, data_size);

  rv = manager.FreeStream(venc_chn_, stream_buf);
  if (rv) return rv;

  rv = manager.FreeVbBuf(venc_chn_, frame_buf);
  return rv;
}

int Websocket::PackSmartMsg(
    x3::FrameMessage &proto_frame_message,
    ai_msgs::msg::PerceptionTargets::SharedPtr smart_msg) {
  float x_ratio = 1.0 * image_width_ / smart_width_;
  float y_ratio = 1.0 * image_height_ / smart_height_;

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

    // rois
    for (auto smart_roi : smart_target.rois) {
      auto proto_box = target->add_boxes_();
      proto_box->set_type_(smart_roi.type);
      auto point1 = proto_box->mutable_top_left_();
      point1->set_x_(smart_roi.rect.x_offset * x_ratio);
      point1->set_y_(smart_roi.rect.y_offset * y_ratio);
      // point1->set_score_(1.0);
      auto point2 = proto_box->mutable_bottom_right_();
      point2->set_x_((smart_roi.rect.x_offset + smart_roi.rect.width) *
                     x_ratio);
      point2->set_y_((smart_roi.rect.y_offset + smart_roi.rect.height) *
                     y_ratio);
      // point2->set_score_(1.0);
      // proto_box->set_score(1.0);
    }

    // attributes
    for (auto smart_attributes : smart_target.attributes) {
      auto attrs = target->add_attributes_();
      attrs->set_type_(smart_attributes.type);
      attrs->set_value_(smart_attributes.value);
      // attrs->set_score_(1.0);
      attrs->set_value_string_(std::to_string(smart_attributes.value));
    }

    // points
    for (auto smart_points : smart_target.points) {
      auto proto_points = target->add_points_();
      proto_points->set_type_(smart_points.type);
      for (auto smart_point : smart_points.point) {
        auto point = proto_points->add_points_();
        point->set_x_(smart_point.x * x_ratio);
        point->set_y_(smart_point.y * y_ratio);
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

int Websocket::SendSmartMessage(
    ai_msgs::msg::PerceptionTargets::SharedPtr smart_msg,
    sensor_msgs::msg::Image::SharedPtr frame_msg) {
  x3::FrameMessage msg_send;
  msg_send.set_timestamp_(frame_msg->header.stamp.sec * 1000000000 +
                          frame_msg->header.stamp.nanosec);
  auto image = msg_send.mutable_img_();
  image->set_buf_((const char *)frame_msg->data.data(), frame_msg->data.size());
  image->set_type_("jpeg");
  image->set_width_(frame_msg->width);
  image->set_height_(frame_msg->height);

  PackSmartMsg(msg_send, smart_msg);

  // add system info
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
  std::string proto_send;
  msg_send.SerializeToString(&proto_send);
  uws_server_->Send(proto_send);
  return 0;
}

void Websocket::MapSmartProc() {
  while (!map_stop_) {
    std::unique_lock<std::mutex> lock(map_smart_mutex_);
    map_smart_condition_.wait(lock);
    if (map_stop_) {
      break;
    }
    while (!x3_smart_msg_.empty() && !x3_frames_.empty()) {
      auto msg = x3_smart_msg_.top();
      auto frame = x3_frames_.top();
      if (msg->header.stamp == frame->header.stamp) {
        int task_num = data_send_thread_.GetTaskNum();
        if (task_num < 3) {
          data_send_thread_.PostTask(
              std::bind(&Websocket::SendSmartMessage, this, msg, frame));
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
