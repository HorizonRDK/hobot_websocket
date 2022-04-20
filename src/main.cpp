// Copyright (c) 2021 Horizon Robotics.All Rights Reserved.
//
// The material in this file is confidential and contains trade secrets
// of Horizon Robotics Inc. This is proprietary information owned by
// Horizon Robotics Inc. No part of this work may be disclosed,
// reproduced, copied, transmitted, or used in any way for any purpose,
// without the express written permission of Horizon Robotics Inc.

#include "rclcpp/rclcpp.hpp"
#include "websocket/websocket.h"

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  auto nh = std::make_shared<rclcpp::Node>("websocket");
  websocket::Websocket web_socket(nh);
  rclcpp::spin(nh);
  rclcpp::shutdown();
}
