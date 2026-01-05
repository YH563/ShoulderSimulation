cmake_minimum_required(VERSION 3.8)
project(shoulder)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_eigen REQUIRED)
find_package(urdf REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(Eigen3 REQUIRED) 
find_package(Sophus REQUIRED) 
find_package(fmt REQUIRED)
find_package(wrench_interfaces REQUIRED) 
find_package(Python3 REQUIRED COMPONENTS Interpreter)

include_directories(
  include
  include/shoulder
  ${EIGEN3_INCLUDE_DIRS}
)

add_executable(shoulder_core
  src/Shoulder.cpp
  src/Nodes/ShoulderCore.cpp  # 新增：Nodes 下的源文件
  src/YAlgorithm/YDynamics.cpp # 新增：YAlgorithm 下的源文件
  src/YAlgorithm/YMotion.cpp
)

ament_target_dependencies(shoulder_core
  rclcpp
  geometry_msgs
  sensor_msgs
  tf2_ros
  tf2_eigen
  urdf
  visualization_msgs
  Eigen3
  Sophus
  wrench_interfaces
)

target_link_libraries(shoulder_core
  ${Sophus_LIBRARIES}
  fmt::fmt  # 链接 fmt 库，注意使用::版本
)

install(TARGETS
  shoulder_core
  DESTINATION lib/${PROJECT_NAME}
)

# 安装launch文件
install(
  DIRECTORY launch/  # 源码中的 launch 目录
  DESTINATION share/${PROJECT_NAME}/launch
)

# 安装URDF文件
install(
  DIRECTORY config/urdf  # 源码中的 urdf 目录
  DESTINATION share/${PROJECT_NAME}/config
)

# 安装Rviz配置文件
install(
  DIRECTORY config/rviz  # 源码中的 config 目录
  DESTINATION share/${PROJECT_NAME}/config
)

install(PROGRAMS
  scripts/Plotter.py
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
