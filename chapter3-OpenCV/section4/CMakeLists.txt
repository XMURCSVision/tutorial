cmake_minimum_required(VERSION 2.8)
project(section4)

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

find_package( OpenCV REQUIRED )

include_directories(include)

# 查找 src 目录下的所有 .cpp 文件，并将其存储在 SOURCES 变量中
file(GLOB SOURCES "src/*.cpp")

include_directories( ${OpenCV_INCLUDE_DIRS} )

# 将所有的源文件添加到可执行文件 detect 中
add_executable(detect ${SOURCES})

target_link_libraries(detect  ${OpenCV_LIBS} )