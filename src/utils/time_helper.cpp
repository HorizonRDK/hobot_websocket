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

#include <sstream>
#include "utils/time_helper.h"

namespace hobot {

std::chrono::time_point<std::chrono::system_clock> Timer::tic() {
  return std::chrono::system_clock::now();
}

double Timer::toc_s(
    const std::chrono::time_point<std::chrono::system_clock> &tic) {
  auto time_d =
      std::chrono::duration_cast<std::chrono::microseconds>(
          std::chrono::system_clock::now() - tic);
  return static_cast<double>(time_d.count()) *
      std::chrono::microseconds::period::num /
      std::chrono::microseconds::period::den;
}

double Timer::toc(
    const std::chrono::time_point<std::chrono::system_clock> &tic) {
  return toc_s(tic) * 1000.0;
}

time_t Timer::current_time_stamp() {
  return std::chrono::duration_cast<std::chrono::milliseconds>(
      std::chrono::system_clock::now().time_since_epoch()).count();
}

}  // namespace hobot
