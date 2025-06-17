cmake_minimum_required(VERSION 3.8)
project(vision)

find_package(ament_cmake REQUIRED)

# 安装配置文件（yaml）和启动文件（launch）
install(DIRECTORY
  config
  launch
  DESTINATION share/${PROJECT_NAME}/
)

ament_package()
