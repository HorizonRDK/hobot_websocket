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

#ifndef TIME_HELPERS_H_
#define TIME_HELPERS_H_

#include <sys/time.h>
#include <string>
#include <chrono>

namespace hobot {
/**
 * @brief The time related helpers.
 */
class Timer {
 public:
  /**
   * @brief Get a clock tic for current time.
   * @return The current time point.
   */
  static std::chrono::time_point<std::chrono::system_clock> tic();

  /**
   * @brief Get the current time stamp.
   * @return The current time stamp.
   */
  static time_t current_time_stamp();

  /**
   * @brief Get the time difference from the tic time.
   * @param tic The tic time.
   * @return The time difference in second.
   */
  static double toc_s(
      const std::chrono::time_point<std::chrono::system_clock> &tic);

  /**
   * @brief Get the time difference from the tic time.
   * @param tic The tic time.
   * @return The time difference in millisecond.
   */
  static double toc(
      const std::chrono::time_point<std::chrono::system_clock> &tic);
};

}  // namespace hobot

#endif  // TIME_HELPERS_H_
