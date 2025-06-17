cmake_minimum_required(VERSION 3.8)
project(node)

# 使用 C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# 查找 ROS 2 依赖包
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(serial_driver REQUIRED)

include_directories(include)

# 添加 publisher 节点
add_executable(publisher
  src/publisher.cpp
)
ament_target_dependencies(publisher
  rclcpp
  std_msgs
  yaml-cpp
  serial_driver
)
target_include_directories(publisher PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

# 添加 subscriber 节点
add_executable(subscriber
  src/subscriber.cpp
)
ament_target_dependencies(subscriber
  rclcpp
  std_msgs
  yaml-cpp
  serial_driver
)
target_include_directories(subscriber PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

# 安装可执行文件
install(TARGETS
  publisher
  subscriber
  DESTINATION lib/${PROJECT_NAME}
)

# 安装头文件
install(DIRECTORY include/
  DESTINATION include/
)

ament_package()

