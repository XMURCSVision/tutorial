cmake_minimum_required(VERSION 3.0)
project(GenShin)

set(CMAKE_CXX_STANDARD 17)

include_directories(include)

find_package(fmt REQUIRED)

add_executable(GenShin src/main.cpp src/genshin.cpp)

target_link_libraries(GenShin fmt::fmt)