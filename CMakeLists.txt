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
  third_party/protobuf/include
  third_party/uWebSockets/include
  third_party/openssl/include
  /usr/include/vio
  ${CMAKE_SYSROOT}/usr/include/vio)

link_directories(
  third_party/protobuf/lib
  third_party/uWebSockets/lib
  third_party/openssl/lib
  /usr/lib/hbmedia
  ${CMAKE_SYSROOT}/usr/lib/hbmedia)
  

add_executable(websocket
  src/main.cpp
  src/websocket/websocket.cpp 
  src/server/uws_server.cpp 
  src/protobuf/x3.pb.cc 
  src/utils/threads/threadpool.cpp
  src/utils/time_helper.cpp
  src/media_codec/media_codec_manager.cpp)

ament_target_dependencies(websocket rclcpp sensor_msgs ai_msgs hbm_img_msgs)
target_link_libraries(websocket uWS ssl crypto z protobuf hbmedia cam vio multimedia isp iar tinyalsa)

install(TARGETS
  websocket
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY
  ${CMAKE_CURRENT_SOURCE_DIR}/webservice
  DESTINATION lib/${PROJECT_NAME})

install(FILES
  ${CMAKE_CURRENT_SOURCE_DIR}/third_party/uWebSockets/lib/libuWS.so
  DESTINATION lib/)

install(FILES
  ${CMAKE_CURRENT_SOURCE_DIR}/third_party/protobuf/lib/libprotobuf.so.31
  DESTINATION lib/)

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

ament_package()
