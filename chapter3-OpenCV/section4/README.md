# 实战：装甲板识别

这一部分参考[rm_vision/rm_auto_aim](https://gitlab.com/rm_vision/rm_auto_aim)

## 自瞄算法

### 图像预处理

图像预处理是计算机视觉和图像处理领域中一个重要的步骤，其目的是增强图像的质量、提取特征、减少噪声或使图像更容易分析和处理。预处理的整体作用
- 灰度化和二值化可以去除颜色信息中的一些噪声，突出图像的结构和形状
- 二值化后的图像可以更容易地提取特征，例如边缘、角点等
- 通过减少颜色通道和图像的灰度级，简化了数据，减少计算资源的需求

在RM比赛中，装甲板存在两个发光的灯条，我们可以利用这个特征检测装甲板。因此，我们需要将灯条在图像中凸显出来。抓住灯条的颜色和亮度特征，可以使用提取单一颜色通道、通道相减、HSV等方法提取特征。

### 灯条识别

由上一步预处理，我们可以得到一张保留灯条明显特征的二值化图像。但是，由于上面只是基于灯条光学特征进行地提取，不可避免会包含其他一些发光（或反光）的对象。为此，我们可以进一步抓住灯条的几何特征进行筛选。
- 长宽比
- 角度

### 灯条匹配

每个装甲板都有两个灯条，这是非常重要的特征之一。找到灯条后，需要将灯条进行两两匹配，以确定哪两个灯条可能属于同一个装甲板。匹配的过程我们还是抓住装甲板灯条的几何特征
- 两个灯条大小是否相近
- 两个灯条角度差是否合理
- 两个灯条的间距是否合适
- 两个灯条组成的装甲板长宽比是否合理

### 数字识别

除了灯条外，装甲板另外一个很重要的特征是装甲板上的数字。这些数字，不仅说明两个灯条匹配是正确的，还表示敌方车辆兵种。数字识别我们用的是简单的MLP网络，通过添加负样本，可以达到非常好的效果。

## 热更新

算法中包含着很多参数，例如二值化阈值、最小长宽比、最大旋转角等。当我们在开发算法初期时，会不断尝试不同的参数。即使在算法完成后，如果算法缺乏鲁棒性，在不同的环境部署时也需要不断尝试调参。如果参数都是写死在源代码中（编译后无法修改参数），或者参数可以在运行时指定，但是参数很多，这会导致运行指令太长。那么调参就会变得很繁琐。因此，我们想到，将参数以固定格式保存在文件中，每次从该文件动态读取即可。

我们使用到的是`yaml-cpp`库。`yaml-cpp`是一个用于解析和生成YAML（YAML Ain't Markup Language）格式文件的C++库。它提供了一种简单而强大的方式来处理YAML数据，使得在C++项目中读取和写入YAML文件变得方便。