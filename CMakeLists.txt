# Copyright (c) 2022，Horizon Robotics.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

cmake_minimum_required(VERSION 3.5)
project(websocket)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(ai_msgs REQUIRED)
find_package(hbm_img_msgs REQUIRED)

include_directories(
  include
  include/websocket
  include/protobuf
  include/utils
)

if (${CMAKE_SYSTEM_PROCESSOR} STREQUAL "aarch64")
  include_directories(
    third_party/aarch64/protobuf/include
    third_party/aarch64/uWebSockets/include
    third_party/aarch64/openssl/include
    /usr/include/vio
    ${CMAKE_SYSROOT}/usr/include/vio
  )

  link_directories(
    third_party/aarch64/protobuf/lib
    third_party/aarch64/uWebSockets/lib
    third_party/aarch64/openssl/lib
    /usr/lib/hbmedia
    ${CMAKE_SYSROOT}/usr/lib/hbmedia
  )
else()
  include_directories(
    third_party/x86/protobuf/include
    third_party/x86/uWebSockets/include
  )

  link_directories(
    third_party/x86/protobuf/lib
    third_party/x86/uWebSockets/lib
  )
endif()

# Add sources
set(SOURCES
  src/main.cpp
  src/websocket/websocket.cpp 
  src/server/uws_server.cpp 
  src/protobuf/x3.pb.cc 
  src/utils/threads/threadpool.cpp
  src/utils/time_helper.cpp
)

# Create executable
add_executable(websocket ${SOURCES})

# Add dependencies
ament_target_dependencies(websocket 
  rclcpp 
  sensor_msgs 
  ai_msgs 
  hbm_img_msgs
)

# Link libraries
target_link_libraries(websocket 
  uWS 
  ssl 
  crypto 
  z 
  libprotobuf.a
)

# Install target
install(TARGETS
  websocket
  DESTINATION lib/${PROJECT_NAME})

# Install webservice
install(DIRECTORY
  ${CMAKE_CURRENT_SOURCE_DIR}/webservice/conf
  ${CMAKE_CURRENT_SOURCE_DIR}/webservice/html
  ${CMAKE_CURRENT_SOURCE_DIR}/webservice/logs
  ${CMAKE_CURRENT_SOURCE_DIR}/webservice/x3
  DESTINATION lib/${PROJECT_NAME}/webservice
)

if (${CMAKE_SYSTEM_PROCESSOR} STREQUAL "aarch64")
  install(FILES
    ${CMAKE_CURRENT_SOURCE_DIR}/third_party/aarch64/uWebSockets/lib/libuWS.so
    DESTINATION lib/)
  install(PROGRAMS
    ${CMAKE_CURRENT_SOURCE_DIR}/webservice/sbin/aarch64/nginx
    DESTINATION lib/${PROJECT_NAME}/webservice/sbin)
else()
  install(FILES
    ${CMAKE_CURRENT_SOURCE_DIR}/third_party/x86/uWebSockets/lib/libuWS.so
    DESTINATION lib/)
  install(PROGRAMS
    ${CMAKE_CURRENT_SOURCE_DIR}/webservice/sbin/x86/nginx
    DESTINATION lib/${PROJECT_NAME}/webservice/sbin)
endif()

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # uncomment the line when a copyright and license is not present in all source files
  #set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # uncomment the line when this package is not in a git repo
  #set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

# Install launch files.
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}/
)

ament_package()
