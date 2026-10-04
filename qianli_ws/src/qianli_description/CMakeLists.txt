cmake_minimum_required(VERSION 3.8)
project(qianli_description)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)

# 模型与配置文件（Milestone 1 起逐步加入 urdf/ meshes/ rviz/）：
#
# install(DIRECTORY urdf rviz
#   DESTINATION share/${PROJECT_NAME}
# )

ament_package()
