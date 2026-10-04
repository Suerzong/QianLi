cmake_minimum_required(VERSION 3.8)
project(qianli_interfaces)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# 新增自定义接口时，将文件放入 msg/ srv/ action/ 目录并取消注释：
#
# find_package(rosidl_default_generators REQUIRED)
# rosidl_generate_interfaces(${PROJECT_NAME}
#   "msg/SemanticEntity.msg"
#   "srv/QuerySemanticMap.srv"
#   "action/Manipulate.action"
# )
# ament_export_dependencies(rosidl_default_runtime)

ament_package()
