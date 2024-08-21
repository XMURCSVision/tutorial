# Python环境配置
在实际项目中，一般都需要使用已经成熟的API，而调用这些API就需要安装相应的库。Anaconda3包含了多个开源软件包，并使用Conda作为包管理器和环境管理器。通过Conda，能够创建和管理独立的环境，使得多个Python环境在一台PC上独立存在互不影响，并智能解决包的依赖与版本问题
## Anaconda3安装与配置
1. [Anaconda3安装](https://blog.csdn.net/scorn_/article/details/106591160)
2. 如果Anaconda3安装好后却不能使用终端命令调出，则说明没有配置好环境变量 : [环境变量配置](https://blog.csdn.net/weixin_45618395/article/details/121120904)
## 虚拟环境
1. 创建：
    ```conda create -n env_name python=x.x```  
    env_name为环境名称，python=x.x为为该环境安装指定版本的python
2. 激活：
    ```conda activate env_name```  
    激活指定的虚拟环境
3. 换源：  
    在进行软件包的安装时，如果不配置镜像源，会使用Anaconda3默认的源，但是这很容易导致连接拒绝访问、连接超时等问题，因此，需要将软件源换成国内的镜像源。  
    1. 临时换源
        ```
        pip install pkg_name -i source
        ```
        pkgname为要安装的包名，source为希望使用的软件源  
    2. 永久换源  
        在C:\Users\你的用户名 中创建pip文件夹
        在pip文件夹中创建pip.ini文件（可使用IDE创建后保存到pip文件夹中）  
        打开pip.ini，输入以下配置(以清华镜像源为例)
        ```
        [global]
        index-url = https://pypi.tuna.tsinghua.edu.cn/simple
        [install]
        trusted-host = pypi.tuna.tsinghua.edu.cn
        ```
    3. 国内常用软件源
        ```
        阿里云
        http://mirrors.aliyun.com/pypi/simple/

        中国科技大学
        https://pypi.mirrors.ustc.edu.cn/simple/

        豆瓣(douban)
        http://pypi.douban.com/simple/

        清华大学
        https://pypi.tuna.tsinghua.edu.cn/simple/

        中国科学技术大学
        http://pypi.mirrors.ustc.edu.cn/simple/
        ```
4. 包的安装  
        ```pip install pkg_name==version```  
        pkg_name为希望安装的包名，==version指定版本(这一步一般省略，由Conda自行安装)，可以使用-i用临时换源的方法安装包。  
        在安装包的过程中经常会遇到各种报错，一般都是网络原因导致的，如果使用镜像源，则需要关闭VPN。如果依然报错，则尝试换网或换其他镜像源。