cmake_minimum_required(VERSION 3.8)
project(qianli_description)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)

# URDF / STL 网格 / RViz 配置 / Launch 全部安装到 share
install(DIRECTORY urdf meshes rviz launch
  DESTINATION share/${PROJECT_NAME})

ament_package()
