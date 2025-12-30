cmake_minimum_required(VERSION 3.8)
project(shoulder_msgs)

# 默认使用C++14，如果你需要C++17，可以修改
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

# 查找依赖
find_package(ament_cmake REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

# 生成消息
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/State.msg"
  DEPENDENCIES std_msgs geometry_msgs
)

# 导出依赖
ament_export_dependencies(rosidl_default_runtime)
ament_export_dependencies(std_msgs)
ament_export_dependencies(geometry_msgs)

# 安装
install(
  DIRECTORY include/
  DESTINATION include
)

install(
  DIRECTORY msg/
  DESTINATION share/${PROJECT_NAME}/msg
)

ament_package()