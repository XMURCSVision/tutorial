cmake_minimum_required(VERSION 2.8)
project(section0)

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

find_package( OpenCV REQUIRED )

include_directories( ${OpenCV_INCLUDE_DIRS} )

add_executable( display src/DisplayImage.cpp )

target_link_libraries( display ${OpenCV_LIBS} )