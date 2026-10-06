cmake_minimum_required(VERSION 3.8)
project(qianli_description)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)

# URDF / STL 网格 / RViz 配置 / Launch 全部安装到 share
install(DIRECTORY urdf meshes rviz launch config scripts
  DESTINATION share/${PROJECT_NAME})

install(FILES README.md DESTINATION share/${PROJECT_NAME})

# Fail the build if generated source assets drift from common.xacro.
find_package(Python3 REQUIRED COMPONENTS Interpreter)
add_custom_target(check_geometry_assets ALL
  COMMAND ${Python3_EXECUTABLE} ${CMAKE_CURRENT_SOURCE_DIR}/scripts/generate_geometry.py --check
  COMMENT "Checking generated QianLi geometry assets")

if(BUILD_TESTING)
  find_package(ament_cmake_test REQUIRED)
  ament_add_test(qianli_geometry
    COMMAND ${Python3_EXECUTABLE} ${CMAKE_CURRENT_SOURCE_DIR}/scripts/validate_geometry.py
    GENERATE_RESULT_FOR_RETURN_CODE_ZERO
    TIMEOUT 60)
endif()

ament_package()
