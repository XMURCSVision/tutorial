# OpenCV

[官方文档4.5.4](https://docs.opencv.org/4.5.4/d9/df8/tutorial_root.html)

## 安装

1. apt安装

```bash
sudo apt install libopencv-dev
```

2. 源码安装

查看opencv版本

```bash
pkg-config --modversion opencv4
```
安装成功（执行上述指令后有版本号输出）后，试着运行文件夹0中的程序。

## image basic

### 图像的存储

OpenCV使用Mat作为图像存储的数据类型，即矩阵。Mat类由图像基本信息和指向图像矩阵地址的指针组成。图像基本信息包括图像大小、图像通道数等。图像像素矩阵不仅和图像形状大小有关，还和图像所使用的颜色空间、图像类型、像素数据类型等相关。

图像通道（Channel）是指图像中的一个数据层，每个层包含一种颜色或强度信息。图像通道可以表示颜色的分量或灰度值。基本上，描述一个像素点，如果是灰度，那么只需要一个数值来描述它，就是单通道。如果一个像素点，有RGB三种颜色来描述它，就是三通道。

颜色空间（Color Space）是一个特定的组织或表示颜色的方法。它定义了一种表示颜色的方式，使得颜色能够被数值化和处理。常见的颜色空间有RGB/BGR、HSV、HSL、YUV。

图像（像素）类型，用于指定每个像素的通道数和通道数值的存储位数。例如`CV_8UC1`表示图像的每个像素是一个无符号8位（8-bit unsigned）的单通道（1 channel）数据类型，`CV_32FC3`表示一种具有3个通道，每个通道使用 32 位浮点数表示的图像类型。

### 基本操作

- Mat

OpenCV提供了一些生成常见矩阵的静态成员函数，如eye，ones，zeros
```cpp
Mat E = Mat::eye(4, 4, CV_64F);   // 单位矩阵
Mat O = Mat::ones(2, 2, CV_32F);  // 全1
Mat Z = Mat::zeros(3,3, CV_8UC1); // 全0
```

```cpp
// 初始化
cv::Mat src = cv::Mat(4, 5, CV_8UC3);
randu(src, cv::Scalar::all(0), cv::Scalar::all(255));

// 基本信息
int rows = src.rows;       
int cols = src.cols;
int channels = src.channels();
cv::Size size = src.size();
cv::Vec3b pixel = src.at<cv::Vec3b>(0, 0);
// uchar pixel = src.at<uchar>(1, 1)

// 修改
cv::Mat dst;
cv::resize(src, dst, cv::Size(10, 12));  // 修改大小
src.at<cv::Vec3b>(1, 0) = cv::Vec3b(1, 2, 3); // 修改像素
// src.at<uchar>(1, 0) = 0

// 拷贝
cv::Mat dst_0(src);
cv::Mat dst_1;  // 浅拷贝
dst_1 = src;    // 浅拷贝
cv::Mat dst_2, dst_3;
dst2 = src.clone();  // 深拷贝
src.copyTo(dst_3);  // 深拷贝
```

## tutorials

以下内容参考[opencv-python中文教程](https://opencv-python-tutorials.readthedocs.io/zh/latest/1.%20OpenCV%E7%AE%80%E4%BB%8B/1.1.%20OpenCV-Python%E6%95%99%E7%A8%8B%E4%BB%8B%E7%BB%8D/)

[section1：OpenCV中的Gui特性](section1/README.md)

[section2：核心操作](section2/README.md)

[section3：OpenCV中的图像处理](section3/README.md)