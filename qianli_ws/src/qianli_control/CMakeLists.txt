cmake_minimum_required(VERSION 3.8)
project(qianli_control)
find_package(ament_cmake REQUIRED)
find_package(Python3 REQUIRED COMPONENTS Interpreter)
install(DIRECTORY config launch DESTINATION share/${PROJECT_NAME})
install(PROGRAMS scripts/test_xdrive_kinematics.py scripts/motion_test.py
  scripts/teleop.py DESTINATION lib/${PROJECT_NAME})
install(FILES README.md DESTINATION share/${PROJECT_NAME})
if(BUILD_TESTING)
  find_package(ament_cmake_test REQUIRED)
  ament_add_test(test_xdrive_kinematics
    COMMAND ${Python3_EXECUTABLE} ${CMAKE_CURRENT_SOURCE_DIR}/scripts/test_xdrive_kinematics.py
    GENERATE_RESULT_FOR_RETURN_CODE_ZERO TIMEOUT 60)
endif()
ament_package()
