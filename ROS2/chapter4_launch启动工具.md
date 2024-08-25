# launch启动工具
## 简介
对于一个机器人系统来说，往往由很多个不同功能的节点组成，启动一个机器人系统时往往需要启动多个节点，同时根据应用场景和机器人的不同，每个节点还会有不同的配置项。

如果每个节点我们都开一个新终端，敲ros2 run指令并写一堆参数，这是多么浪费生命且令人绝望的事情。

除了启动，你会发现，一个个关闭也是很难受的。

因此launch文件应运而生。launch文件允许我们同时启动和配置多个包含 ROS2 节点的可执行文件。ROS2中的launch文件允许以python、xml、yaml三种格式书写，我们主要学习其python写法
## 节点启动工具
1. 在功能包目录中创建launch文件夹，其与src同级
2. 创建对应的launch文件，后缀为.launch.py
3. 导入库：
    - LaunchDescription，用于对launch文件内容进行描述
    - 是Node，用于声明节点所在的位置。
    ```python
    from launch import LaunchDescription
    from launch_ros.actions import Node
    ```
4. 实例化节点对象：第一个参数为功能包名称，第二个参数为可执行文件名称
    ```python
    pub = Node(
        package="example02_topic",
        executable="my_pub"
    )
    sub = Node(
        package="example02_topic",
        executable="my_sub"
    )
    ```
5. 实例化launch描述对象：传入一个节点对象组成的列表，列表中的节点将随launch文件被启动
    ```python
    launch_description = LaunchDescription(
    [pub, sub])
    ```
6. 返回launch描述对象
    ```python
    return launch_description
    ```
7. 完整源码：
    ```python
    # 导入库
    from launch import LaunchDescription
    from launch_ros.actions import Node

    def generate_launch_description():
        """launch内容描述函数，由ros2 launch 扫描调用"""
        pub = Node(
            package="example02_topic",
            executable="my_pub"
        )
        sub = Node(
            package="example02_topic",
            executable="my_sub"
        )
        # 创建LaunchDescription对象launch_description,用于描述launch文件
        launch_description = LaunchDescription(
            [pub, sub])
        # 返回让ROS2根据launch描述执行节点
        return launch_description
    ```
8. 修改Cmake文件，将launch目录下文件安装到编译生成的install目录下
    ```cmake
    install(DIRECTORY launch
    DESTINATION share/${PROJECT_NAME})
    ```
## 添加参数&修改命名空间
launch文件不仅可以启动指定节点，也可以向节点中传入参数和给节点以不同的命名空间。我们目前只需要学习传入参数的功能
1. 修改源码，
    ```cpp
    public:
        this->declare_parameter<std::string>("fishtype", "cao");
        fishtype = this->get_parameter("fishtype").as_string();
        this->declare_parameter<int>("heavy", 30);
        heavy = this->get_parameter("heavy").as_int();
    private:
        std::string fishtype;
        int heavy;
    ```
2. 修改launch文件，在实例化Node时传入parameter参数，如：
    ```python
        pub = Node(
        package="example02_topic",
        executable="my_pub",
        parameters = [
            {'fishtype' : 'goldfish'},
            {'heavy' : 10}
        ]
    )
    ```

# 任务
1. 编写一个launch文件，用于启动上一章任务中机器人发布者和订阅者节点
2. 在launch文件中传入机器人速度和坐标的初始值