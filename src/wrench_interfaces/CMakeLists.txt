cmake_minimum_required(VERSION 3.8)
# 注意：项目名必须和package.xml中的name一致（wrench_interfaces）
project(wrench_interfaces)

# 编译器警告选项（可选，保留即可）
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# 查找依赖
find_package(ament_cmake REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(rosidl_default_generators REQUIRED)

# 生成服务接口（核心步骤）
rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/WrenchData.srv"  # 确保srv文件夹下有这个文件
  DEPENDENCIES builtin_interfaces  # 依赖builtin_interfaces
)

# 导出运行时依赖（必须）
ament_export_dependencies(rosidl_default_runtime)
ament_export_dependencies(builtin_interfaces)

# 测试部分（保留，不影响核心功能）
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()