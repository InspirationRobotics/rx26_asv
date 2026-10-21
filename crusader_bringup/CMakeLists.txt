cmake_minimum_required(VERSION 3.8)
project(crusader_bringup)

find_package(ament_cmake REQUIRED)

# Data-only package: ament_cmake rather than ament_python because there is
# no python module here at all, and a setup.py with packages=[] is a puzzle
# for the next person. launch/ and config/ land in share/crusader_bringup/.
install(DIRECTORY launch config DESTINATION share/${PROJECT_NAME})

ament_package()
