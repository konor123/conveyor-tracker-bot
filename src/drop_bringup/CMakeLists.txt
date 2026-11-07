cmake_minimum_required(VERSION 3.8)
project(drop_bringup)

find_package(ament_cmake REQUIRED)

install(DIRECTORY launch config DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(test_mock_bringup test/test_mock_bringup.py TIMEOUT 120)
endif()

ament_package()
