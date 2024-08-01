# linux基础

[Linux 教程 | 菜鸟教程](https://www.runoob.com/linux/linux-tutorial.html)

## 安装Linux系统

- 双系统
- 虚拟机（推荐）
- WSL（推荐）
- Docker

## Linux入门与基础知识

 Linux 是一个开源的操作系统，由 Linus Torvalds 在 1991 年首次发布。它基于 Unix 系统，广泛用于服务器、桌面电脑和嵌入式设备。常见的 Linux 发行版，如 Ubuntu、Fedora、Debian、CentOS 等。

相比于Windows，Linux缺少美丽的图形化交互界面，大部分是在黑漆漆的终端通过命令来进行交互。

## Linux基本命令与文件操作

1. 基本命令
    
    ```bash
    ls      # 列出目录及文件名
    cd      # 切换目录
    pwd     # 显示当前的目录
    mkdir   # 创建一个新的目录
    rm      # 删除文件或目录
    cp      # 复制文件或目录
    mv      # 移动文件与目录，或修改文件与目录的名称
    cat     # 由第一行开始显示文件内容
    ```
    
2. vim、nano、gedit
    
    文本编辑器
    
3. apt
    
    apt（Advanced Packaging Tool）是一个在 Debian 和 Ubuntu 中的 Shell 前端软件包管理器。
    
    apt 命令提供了查找、安装、升级、删除某一个、一组甚至全部软件包的命令，而且命令简洁而又好记。
    
    apt 命令执行需要超级管理员权限(root)。
    
    ```bash
    sudo apt update    # 列出所有可更新的软件清单
    sudo apt upgrade   # 升级软件包
    sudo apt install <package_name>   # 安装指定软件
    ```
    

## 常用工具

- Git：版本管理工具
- VSCode：代码编辑器，配上强大的插件，yyds
- CMake：编译工具
- ssh：远程登入
- nomachine：远程桌面
- AI：chatgpt、kimi、copilot等

## Task

- [ ]  安装Linux系统
- [ ]  尝试科学上网
- [ ]  使用AI工具
- [ ]  尝试linux指令，并在`~/tutorial/ch2/1`下创建文件`main.cpp`，编写一段Hello World程序