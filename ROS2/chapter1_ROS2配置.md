# 什么是ROS，为什么是ROS
1. ROS（Robot Operating System）和ROS2是用于机器人软件开发的**开源框架**。它们提供了一系列工具和库，帮助开发者构建复杂的机器人应用。
2.    
    设想我们的自瞄任务框架：装甲板检测、运动预测、与电控串口通信、可视化调试...  
    如何将各个模块组成一整个项目？使用类？使用多线程？使用某种通信机制？
    每个模块相互关联，却又不像类一样联系紧密；而多线程的方法又不稳定；因此，使用通信的方法将各个模块结合起来。  
    ROS和ROS2提供了这样一个通信机制，允许不同模块之间通过多种方式进行通信，其最重要的性质之一便是项目解耦
3.   
    同时，ROS和ROS2已经形成了较为成熟的生态，有众多完善而稳定的API、软件和开源项目，借助其开源社区，能让项目的开发事半功倍。

4. 
   顾名思义，ROS2是ROS的后续版本，在诸多方面都进行了改进，感兴趣的同学可以自行了解。
  
# 安装ROS2-humble
ROS2框架需要安装在Linux操作系统中。Linux虚拟机的安装，锦标学长已经教过，在此不赘述，尚未安装的同学可以翻看**Linux基础与虚拟机安装.pdf**查阅。以WSL2为例，进行ROS2的安装：
1. 使用SSH远程连接WSL2。推荐使用VSCode进行（远程资源管理器-WSL目标）。
2. 在终端输入：
    ```sh
    sudo wget http://fishros.com/install -O fishros && . fishros
    ```
    随后根据指示安装ROS2-humble。选择桌面版，安装时间一般较长。
3. 安装过程中若出现报错，首先查看报错原因，并检查终端输入是否正确，若输入无误，则大概率为网络问题，换时间换地点重试几次即可。  
4. 检查是否成功安装：重启终端，然后在终端输入`ros2`，若显示`Call 'ros2 <command> -h' for more detailed usage.`，则安装成功。

# ROS2环境变量设置：source命令 与 setup.bash文件
在ROS2中，setup.bash文件是一个重要的环境配置脚本，用于设置ROS2的环境变量，以便在终端中使用ROS2的命令和功能。使用source将脚本中的环境配置写入终端中。
1. 环境变量设置：setup.bash文件负责设置ROS2所需的环境变量，使得系统能够找到安装的ROS2包和工具。例如，执行以下命令可以加载ROS2的环境：
    ```sh
    source /opt/ros/humble/setup.bash
    ```
    source setup.bash即为运行bash文件，类似于`python name.py`运行Python文件。  
    当然，你也可以自己写一个setup.bash文件。  
    注：
      - 若桌面操作出现权限问题，使用`sudo touch setup.bash`创建文件。**touch file_name**是一个Linux命令，用于创建文件。sudo为使用超级用户权限执行命令。
      -  Linux文本编辑器包括nano、gedit等等，我个人比较喜欢gedit，可以使用`sudo apt install gedit`安装该文本编辑器。也可以在VSCode中直接打开编辑.
        ```sh
        #!/usr/bin/env bash
        echo "hello, i am setup.bash, now your source is done!"
        ```

2. 持久化配置：为了避免每次打开新终端时都需要手动执行上述命令，用户可以将其添加到`~/.bashrc`文件中。该文件会在每次终端打开时自动执行：
    ```sh
    echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
    ```
    这样，每次打开新的终端时，系统会自动加载ROS2的环境设置。  
    也可以使用：
    ```sh
    gedit ~/.bashrc
    ```
    直接编辑该文件。
# 任务：
- 完成ROS2-humble的安装
- 自主体验bash文件相关操作
