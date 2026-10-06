cmake_minimum_required(VERSION 3.8)
project(qianli_omni_learning)
find_package(ament_cmake REQUIRED)
install(DIRECTORY scripts config policies DESTINATION share/${PROJECT_NAME})
install(DIRECTORY scripts/ DESTINATION lib/${PROJECT_NAME}
  FILES_MATCHING PATTERN "*.py"
  PERMISSIONS OWNER_READ OWNER_WRITE OWNER_EXECUTE GROUP_READ GROUP_EXECUTE WORLD_READ WORLD_EXECUTE)
install(FILES README.md DESTINATION share/${PROJECT_NAME})
ament_package()
