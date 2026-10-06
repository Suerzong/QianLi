cmake_minimum_required(VERSION 3.8)
project(qianli_exploration)
find_package(ament_cmake REQUIRED)
install(DIRECTORY launch config DESTINATION share/${PROJECT_NAME})
install(DIRECTORY scripts/ DESTINATION lib/${PROJECT_NAME}
  FILES_MATCHING PATTERN "*.py"
  PERMISSIONS OWNER_READ OWNER_WRITE OWNER_EXECUTE GROUP_READ GROUP_EXECUTE WORLD_READ WORLD_EXECUTE)
install(FILES README.md interfaces.md TODO.md DESTINATION share/${PROJECT_NAME})
if(BUILD_TESTING)
  find_package(ament_cmake_test REQUIRED)
  find_package(Python3 REQUIRED COMPONENTS Interpreter)
  ament_add_test(frontier_core COMMAND "${Python3_EXECUTABLE}" "${CMAKE_CURRENT_SOURCE_DIR}/test/test_frontier_core.py"
    GENERATE_RESULT_FOR_RETURN_CODE_ZERO TIMEOUT 60)
  ament_add_test(frontier_lifecycle COMMAND "${Python3_EXECUTABLE}" "${CMAKE_CURRENT_SOURCE_DIR}/test/test_frontier_lifecycle.py"
    GENERATE_RESULT_FOR_RETURN_CODE_ZERO TIMEOUT 60)
  set_tests_properties(frontier_lifecycle PROPERTIES ENVIRONMENT "ROS_DOMAIN_ID=82")
endif()
ament_package()
