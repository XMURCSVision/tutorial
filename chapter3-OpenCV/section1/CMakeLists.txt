cmake_minimum_required(VERSION 2.8)
project(section1)

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

find_package( OpenCV REQUIRED )

include_directories( ${OpenCV_INCLUDE_DIRS} )

add_executable(image src/image.cpp)
add_executable(video src/video.cpp)
add_executable(draw src/draw.cpp)

target_link_libraries(image  ${OpenCV_LIBS} )
target_link_libraries(video  ${OpenCV_LIBS} )
target_link_libraries(draw ${OpenCV_LIBS} )