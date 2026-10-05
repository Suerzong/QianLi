cmake_minimum_required(VERSION 3.8)
project(qianli_training_scenarios)
find_package(ament_cmake REQUIRED)
install(DIRECTORY generated scripts rviz DESTINATION share/${PROJECT_NAME})
install(PROGRAMS scripts/benchmark.py scripts/plan_suite.py scripts/run_vm_session.py
  DESTINATION lib/${PROJECT_NAME})
install(FILES README.md DESTINATION share/${PROJECT_NAME})
ament_package()
