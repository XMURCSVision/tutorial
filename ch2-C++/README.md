# c++

## Linux-GenShin Start

1. 单个文件编译
    1. 准备好genshin.cpp文件
        
        ```cpp
        #include <iostream>
        
        int main() {
          std::cout << "GenShin Start!" << std::endl;
          return 0;
        }
        ```
        
    2. 编译
        
        ```bash
        g++ genshin.cpp -o genshin
        ```
        
2. 多个文件编译
    1. 准备好相应文件
        
        ```bash
        .
        ├── include
        │   └── genshin.h
        └── src
            ├── genshin.cpp
            └── main.cpp
        ```
        
        genshin.h
        
        ```cpp
        #ifndef GENSHIN_H
        #define GENSHIN_H
        
        #include <iostream>
        
        void genshin_start();
        
        #endif  // GENSHIN_H
        ```
        
        genshin.cpp
        
        ```cpp
        #include <iostream>
        
        #include "genshin.h"
        
        void genshin_start() { std::cout << "GenShin Start!" << std::endl; }
        ```
        
        main.cpp
        
        ```cpp
        #include "genshin.h"
        
        int main() {
          genshin_start();
          return 0;
        }
        ```
        
    2. 编译
        
        ```bash
        g++ ./src/main.cpp ./src/genshin.cpp -I ./include/ -o main
        ```
        
    
3. 使用第三方库
    1. 准备好相应文件
    2. 编译
        
        ```bash
        g++ ./src/main.cpp ./src/genshin.cpp -I ./include/ -o main -lfmt
        ```
        

如果项目大起来，涉及到很多cpp文件，或者需要大量调用第三方库，这样的编译方式会让你在寻找头文件上消耗巨大量的精力，因此，我们需要一个工具，来帮助我们简化需要依赖的过程。

## CMake

CMake 是一个开源的、跨平台的构建系统，它用于管理项目的构建过程。与传统的构建系统（如 Make）相比，CMake 提供了更高级的功能和灵活性。它通过平台无关的配置文件（CMakeLists.txt）来定义构建过程，然后生成特定平台的构建文件（如 Makefile 或 Visual Studio 工程文件）。

1. 组织项目结构
    
    ```bash
    .
    ├── CMakeLists.txt
    ├── include
    │   └── genshin.h
    └── src
        ├── genshin.cpp
        └── main.cpp
    ```
    
2. 编写CMakeLists.txt
    
    ```makefile
    # 设置CMake的最低版本要求
    cmake_minimum_required(VERSION 3.0)
    # 设置项目名称
    project(GenShin)
    
    # 设置c++标准
    set(CMAKE_CXX_STANDARD 17)
    # 设置生成compile_commands.json文件
    set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
    
    # 包含include目录
    include_directories(include)
    
    # 查找fmt库
    find_package(fmt REQUIRED)
    
    # 指定要生成的可执行文件
    add_executable(GenShin src/main.cpp src/genshin.cpp)
    
    # 链接fmt库
    target_link_libraries(GenShin fmt::fmt)
    ```
    
3. 编译
    
    ```bash
    mkdir build  && cd build
    cmake ..
    make
    ```
    

## C++

[C++ 教程 | 菜鸟教程](https://www.runoob.com/cplusplus/cpp-tutorial.html)

- 命名空间namespace
- 引用
- 类 & 对象
- 继承
- 重载
- 封装
- STL库
- 内存管理
- 智能指针
- 多线程

## 代码规范

[C++ 风格指南 - 内容目录 — Google 开源项目风格指南](https://zh-google-styleguide.readthedocs.io/en/latest/google-cpp-styleguide/contents.html)

## Task

- [ ]  尝试给genshin程序打断点调试
- [ ]  重构文件夹5中的项目，并使用cmake进行构建（重构前请按照下面的指令安装依赖并编译运行）


```bash
# 安装依赖
sudo apt install libgl1-mesa-dev libglu1-mesa-dev freeglut3-dev -y

# 编译
g++ -o test main.cpp -lGL -lGLU -lglut

# 运行
./test
```