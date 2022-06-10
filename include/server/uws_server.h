// Copyright (c) 2022，Horizon Robotics.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.
#ifndef INCLUDE_WEBSOCKETPLUGIN_SERVER_UWS_SERVER_H_
#define INCLUDE_WEBSOCKETPLUGIN_SERVER_UWS_SERVER_H_
#include <memory>
#include <string>
#include <thread>

#include "uWS/uWS.h"


namespace websocket {

class UwsServer {
 public:
  explicit UwsServer(const std::string &config);
  UwsServer() : connetion_(nullptr), worker_(nullptr) {}
  ~UwsServer() = default;

 public:
  int Init();
  int Send(const std::string &protocol);
  int DeInit();

 private:
  void StartServer();
  std::mutex mutex_;
  uWS::WebSocket<uWS::SERVER> *connetion_;
  std::shared_ptr<std::thread> worker_;
 private:
  std::string config_file_;
};
}  // namespace websocket

#endif  // INCLUDE_WEBSOCKETPLUGIN_SERVER_UWS_SERVER_H_
