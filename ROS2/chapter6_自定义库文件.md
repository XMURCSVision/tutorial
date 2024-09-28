# hpp书写
在功能包的include目录下，新建hpp文件，在该文件中声明各种变量和类
```cpp
#ifndef EXAMPLE04_A_STAR__A_STAR_HPP_
#define EXAMPLE04_A_STAR__A_STAR_HPP_

class Astar
{
public:
  Astar();
  ~Astar();

  void say_hello();

private:
  int example_variable_;
};

#endif 
```
我们也可以声明其命名空间
```cpp
#ifndef EXAMPLE04_A_STAR__A_STAR_HPP_
#define EXAMPLE04_A_STAR__A_STAR_HPP_

namespace space_name
{
    class Astar
    {
    public:
    Astar();
    ~Astar();

    void say_hello();

    private:
    int example_variable_;
    };
}

#endif 
```
在C++中，#ifndef、#define 和 #endif 是用于防止头文件被多次包含的预处理指令。这些指令可以避免头文件重复定义或重复包含所导致的编译错误。

- #ifndef (If Not Defined):
#ifndef是“if not defined”的缩写。它的作用是检查某个宏（宏通常是用#define定义的）是否已经定义。
如果宏尚未定义，编译器将会处理#ifndef和对应的#endif之间的代码。

- #define:
#define指令用于定义一个宏。在#ifndef之后，它用于定义一个标识符（通常是头文件名的大写形式），表明这个头文件已经被处理。

- #endif:
#endif用于结束从#ifndef开始的条件编译块。它的作用是告诉编译器结束#ifndef的作用范围。

# hpp文件中变量的定义
在src新建一个cpp文件，用于定义hpp中声明的各种变量
```cpp
#include "examole04_a_star/a_star.hpp"
#include <iostream>

Astar::Astar() : example_variable_(0) {
  // 构造函数
}

Astar::~Astar() {
  // 析构函数
}

void Astar::say_hello() {
  std::cout << "Hello, ROS2!" << std::endl;
}
```

# main.cpp的书写
最终调用的程序示例如下
```cpp
#include "examole04_a_star/a_star.hpp"
#include <rclcpp/rclcpp.hpp>

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);

  auto node = rclcpp::Node::make_shared("example_node");
  Astar example;
  example.say_hello();

  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
```

# CmakeLists的书写
```CMAKE
cmake_minimum_required(VERSION 3.5)
project(examole04_a_star)

# 查找依赖项
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)

# 包含目录
include_directories(include)

# 添加库
add_library(${PROJECT_NAME} src/a_star.cpp)

# 链接库
ament_target_dependencies(${PROJECT_NAME} rclcpp)

# 安装
install(TARGETS
  ${PROJECT_NAME}
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include/
)

# 添加可执行文件
add_executable(example_node src/main.cpp)

# 链接库
target_link_libraries(example_node ${PROJECT_NAME})

# 安装可执行文件
install(TARGETS example_node
  DESTINATION lib/${PROJECT_NAME}
)

# 使包能够通过colcon构建
ament_package()

```
其中
- include_directories指定编译器在编译源文件时需要搜索的目录。在这里，指定了include目录，该目录通常包含项目的头文件（.hpp文件）。通过这样做，头文件可以被源文件正确引用
- add_library命令用于添加一个库目标。在这里，将库命名为${PROJECT_NAME}（即examole04_a_star），并且该库的源文件是src/a_star.cpp。
这样做的好处是将代码分离为可复用的库，可以在同一项目的其他部分或其他项目中链接和使用
- install命令用于定义目标文件的安装规则。在这里，将```${PROJECT_NAME}```（即examole04_a_star）库安装到```${CMAKE_INSTALL_PREFIX}/lib/${PROJECT_NAME}```目录中。
默认的CMAKE_INSTALL_PREFIX在ROS 2中是安装空间，例如install目录
- target_link_libraries命令用于将目标链接到库。在这里，将example_node可执行文件链接到${PROJECT_NAME}（即examole04_a_star）库。
这确保了可执行文件可以访问库中定义的类和函数。

# 任务
- 建立两个功能包，第一个功能包发布机器人当前的位置信息；第二个功能包中，在hpp库文件声明A*算法类，在一个cpp文件中实现```A*```算法类，在另一个cpp文件中订阅机器人当前的位置信息并使用```A*```算法求到目标点的路径，最后将路径发布。目标点由launch文件给出，地图在main.cpp中定义。机器人坐标使用geometry_msgs::Pose描述，其orientation均令其为0.查看该消息的API：https://docs.ros.org/en/ros2_packages/rolling/api/geometry_msgs/interfaces/msg/Pose.html
