cmake_minimum_required(VERSION 2.8)
project(section2)

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

find_package( OpenCV REQUIRED )

include_directories( ${OpenCV_INCLUDE_DIRS} )

add_executable(mask src/mask.cpp)

target_link_libraries(mask  ${OpenCV_LIBS} )