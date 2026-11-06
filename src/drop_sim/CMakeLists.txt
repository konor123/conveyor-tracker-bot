cmake_minimum_required(VERSION 3.8)
project(drop_sim)

find_package(ament_cmake REQUIRED)

install(DIRECTORY usd scripts config DESTINATION share/${PROJECT_NAME})

ament_package()
