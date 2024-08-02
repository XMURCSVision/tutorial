# OpenCV中的图像处理

## 改变颜色空间

OpenCV中有多种颜色空间转换方法。我们只研究两种最广泛使用的转换方法，BGR↔Gray和BGR↔HSV。 对于颜色转换，使用函数`cv::cvtColor()`，其中flag确定转换类型。 对于BGR→Gray转换，我们使用标志`cv::COLOR_BGR2GRAY`。类似地，对于BGR→HSV，我们使用标志`cv::COLOR_BGR2HSV`。

对于HSV色彩空间，Hue(色调)的取值范围是[0,179]，Saturation(饱和度)的取值范围是[0,255]，Value(明度)的取值范围是[0,255]。不同的软件可能使用不同的取值方式，因此，如果要将OpenCV的HSV值与其他软件的HSV值进行比较时，则需要对这些范围进行标准化。

使用HSV色彩空间来提取彩色对象
```cpp
// 将图像从BGR转换为HSV
cv::Mat hsvImage;
cv::cvtColor(image, hsvImage, cv::COLOR_BGR2HSV);

// 定义红色的HSV范围（这些值可以根据需要调整）
cv::Scalar lowerRed1(0, 100, 100);
cv::Scalar upperRed1(10, 255, 255);
cv::Scalar lowerRed2(160, 100, 100);
cv::Scalar upperRed2(179, 255, 255);

// 创建掩模，只保留红色部分
cv::Mat mask1, mask2, redMask;
cv::inRange(hsvImage, lowerRed1, upperRed1, mask1);
cv::inRange(hsvImage, lowerRed2, upperRed2, mask2);
cv::bitwise_or(mask1, mask2, redMask);

// 使用掩模提取红色对象
cv::Mat redObjects;
cv::bitwise_and(image, image, redObjects, redMask);
```
## 图像的几何变换

OpenCV提供了两个转换函数`cv::warpAffine`和`cv::warpPerspective`，你可以使用它们进行各种转换。`cv::warpAffine`采用2x3变换矩阵作为参数输入，而`cv::warpPerspective`采用3x3变换矩阵作为参数输入。
```cpp
void cv::warpAffine(
    InputArray src,         // 输入图像
    OutputArray dst,        // 输出图像
    InputArray M,           // 仿射变换矩阵
    Size dsize,             // 输出图像的大小
    int flags = INTER_LINEAR,       // 插值方法
    int borderMode = BORDER_CONSTANT, // 边界模式
    const Scalar& borderValue = Scalar() // 边界填充值
);

void cv::warpPerspective(
    InputArray src,         // 输入图像
    OutputArray dst,        // 输出图像
    InputArray M,           // 透视变换矩阵
    Size dsize,             // 输出图像的大小
    int flags = INTER_LINEAR,       // 插值方法
    int borderMode = BORDER_CONSTANT, // 边界模式
    const Scalar& borderValue = Scalar() // 边界填充值
);
```

- 缩放

缩放只是调整图像大小，OpenCV有一个函数cv::resize()，可以手动指定图像的大小以及缩放系数，可以使用不同的插值方法，常用的插值方法是用于缩小的cv::INTER_AREA和用于缩放的cv::INTER_CUBIC（慢）和cv::INTER_LINEAR。默认情况下，使用的插值方法是cv::INTER_LINEAR，它用于所有调整大小的操作。
```cpp
cv::Mat resizedImage;
cv::resize(image, resizedImage, cv::Size(), scaleX, scaleY);
```

- 平移

```cpp
cv::Mat translationMatrix = (cv::Mat_<double>(2,3) << 1, 0, tx, 0, 1, ty);  // tx、ty表示平移量
cv::Mat translatedImage;
cv::warpAffine(image, translatedImage, translationMatrix, image.size());
```

- 旋转

```cpp
cv::Point2f center(image.cols / 2.0, image.rows / 2.0);
double angle = 45.0;
double scale = 1.0;
cv::Mat rotationMatrix = cv::getRotationMatrix2D(center, angle, scale);
cv::Mat rotatedImage;
cv::warpAffine(image, rotatedImage, rotationMatrix, image.size());
```

- 翻转

```cpp
cv::Mat flippedImage;
cv::flip(image, flippedImage, 1);  // 水平翻转
```

- 仿射变换

在仿射变换中，原始图像中的所有平行线在输出图像中依旧平行。适用于没有视角变化的几何变换。为了找到变换矩阵，我们需要从输入图像中得到三个点，以及它们在输出图像中的对应位置。然后`cv::getAffineTransform`将创建一个2x3矩阵，最后该矩阵将传递给`cv::warpAffine`。
```cpp
cv::Point2f srcTri[3];
cv::Point2f dstTri[3];

// 原图中的三点
srcTri[0] = cv::Point2f(0, 0);
srcTri[1] = cv::Point2f(image.cols - 1, 0);
srcTri[2] = cv::Point2f(0, image.rows - 1);

// 目标图中的三点
dstTri[0] = cv::Point2f(0, image.rows * 0.33);
dstTri[1] = cv::Point2f(image.cols * 0.85, image.rows * 0.25);
dstTri[2] = cv::Point2f(image.cols * 0.15, image.rows * 0.7);

// 获取仿射变换矩阵
cv::Mat affineMatrix = cv::getAffineTransform(srcTri, dstTri);
cv::Mat affineImage;
cv::warpAffine(image, affineImage, affineMatrix, image.size());
```

- 透视变换

透视变换不保留平行性，但保持投影关系，能够模拟人眼或相机的透视效果，使得远处的物体显得更小，近处的物体显得更大。适用于带有视角变化的场景，如透视校正、3D变换。对于透视变换，需要一个3x3变换矩阵。即使在转换之后，直线仍是直线。要找到此变换矩阵，需要在输入图像上找4个点，以及它们在输出图像中的对应位置。在这4个点中，其中任意3个不共线。然后可以通过函数`cv::getPerspectiveTransform`找到变换矩阵，将`cv::warpPerspective`应用于此3x3变换矩阵。
```cpp
cv::Point2f srcQuad[4];
cv::Point2f dstQuad[4];

// 原图中的四点
srcQuad[0] = cv::Point2f(0, 0);
srcQuad[1] = cv::Point2f(image.cols - 1, 0);
srcQuad[2] = cv::Point2f(image.cols - 1, image.rows - 1);
srcQuad[3] = cv::Point2f(0, image.rows - 1);

// 目标图中的四点
dstQuad[0] = cv::Point2f(50, 50);
dstQuad[1] = cv::Point2f(image.cols - 100, 50);
dstQuad[2] = cv::Point2f(image.cols - 50, image.rows - 50);
dstQuad[3] = cv::Point2f(100, image.rows - 50);

// 获取透视变换矩阵
cv::Mat perspectiveMatrix = cv::getPerspectiveTransform(srcQuad, dstQuad);
cv::Mat perspectiveImage;
cv::warpPerspective(image, perspectiveImage, perspectiveMatrix, image.size());
```

## 图像阈值

- 简单阈值处理

这种阈值处理的方法是简单易懂的。如果像素值大于阈值，则为其分配一个值（可以是白色），否则为其分配另一个值（可以是黑色）。使用的函数是cv::threshold。函数第一个参数是源图像，它应该是灰度图像。第三个参数是用于对像素值进行分类的阈值。第四个参数是maxVal，它表示如果像素值大于（有时小于）阈值则要给出的值。OpenCV提供不同类型的阈值，由函数的第五个参数决定。
```cpp
double cv::threshold(
    InputArray src,         // 输入图像
    OutputArray dst,        // 输出图像
    double thresh,          // 阈值
    double maxval,          // 最大值
    int type                // 阈值类型
);	
```

- 自适应阈值处理

在上面，我们使用全局值作为阈值，但在图像在不同区域具有不同照明条件的所有条件下可能并不好。在那种情况下，我们进行自适应阈值处理，算法计算图像的小区域的阈值，所以我们对同一幅图像的不同区域给出不同的阈值，这给我们在不同光照下的图像提供了更好的结果。
```cpp
void cv::adaptiveThreshold(
    InputArray src,         // 输入图像
    OutputArray dst,        // 输出图像
    double maxValue,        // 最大值
    int adaptiveMethod,     // 自适应方法
    int thresholdType,      // 阈值类型
    int blockSize,          // 邻域大小
    double C                // 常数
);
```

- Otsu's二值化

Otsu's 二值化是一种自动确定图像阈值的二值化方法，通过最大化类间方差来找到最佳阈值，使得背景和前景尽可能分开。它是一种自适应的阈值选择方法，不需要手动设定阈值。

Otsu's 方法通过最大化类间方差（即前景和背景的像素值差异）来自动选择最佳的阈值。这种方法假设图像包含两个类（前景和背景），并尝试找到一个阈值，将这两个类区分开来，使得每个类内的像素强度值尽可能的相似，而类间的像素强度差异最大。
```cpp
double otsuThresh = cv::threshold(src, dst, 0, 255, cv::THRESH_BINARY | cv::THRESH_OTSU);
```

## 图像滤波

- 2D卷积（图像过滤）

与一维信号一样，图像也可以使用各种低通滤波器（LPF），高通滤波器（HPF）等进行滤波。LPF有助于消除噪声，模糊图像等。HPF滤波器有助于找到图片的边缘。OpenCV提供了一个函数`cv::filter2D()`来将卷积核与图像进行卷积。
```cpp
void cv::filter2D(
    InputArray src,         // 输入图像
    OutputArray dst,        // 输出图像
    int ddepth,             // 输出图像的深度
    InputArray kernel,      // 卷积核
    Point anchor = Point(-1, -1), // 锚点位置
    double delta = 0,       // 额外加到结果上的偏移量
    int borderType = BORDER_DEFAULT // 边界模式
);
```

- 图像模糊（图像平滑）

通过将图像与低通滤波器卷积核卷积来实现平滑图像。它有助于消除噪音，从图像中去除了高频内容（例如：噪声，边缘）。因此在此操作中边缘会模糊一点。（有的平滑技术也不会平滑边缘）。OpenCV主要提供四种平滑技术。
```cpp
// 均值模糊: 简单平均，计算速度快，但会模糊边缘。
void cv::blur(
    InputArray src,          // 输入图像
    OutputArray dst,         // 输出图像
    Size ksize,              // 内核大小
    Point anchor = Point(-1,-1), // 锚点位置，默认是内核中心
    int borderType = BORDER_DEFAULT // 边界模式
);

// 高斯模糊: 权重中心的模糊，通常用来减少高斯噪声。
void cv::GaussianBlur(
    InputArray src,          // 输入图像
    OutputArray dst,         // 输出图像
    Size ksize,              // 内核大小
    double sigmaX,           // X 方向的标准差
    double sigmaY = 0,       // Y 方向的标准差，如果为 0 则 sigmaY = sigmaX
    int borderType = BORDER_DEFAULT // 边界模式
);

// 中值模糊: 对椒盐噪声非常有效，保留边缘。
void cv::medianBlur(
    InputArray src,          // 输入图像
    OutputArray dst,         // 输出图像
    int ksize                // 内核大小，必须是奇数
);

// 双边滤波: 能模糊图像同时保留边缘，是一种非线性滤波器。
void cv::bilateralFilter(
    InputArray src,          // 输入图像
    OutputArray dst,         // 输出图像
    int d,                   // 过滤过程中每个像素的邻域直径
    double sigmaColor,       // 颜色空间滤波器的标准差
    double sigmaSpace,       // 坐标空间滤波器的标准差
    int borderType = BORDER_DEFAULT // 边界模式
);
```

## 形态变换

形态学转换是基于图像形状的一些简单操作。它通常在二进制图像上执行。它需要两个关键输入参数，一个是我们的原始图像，第二个是称为结构元素或核，它决定了操作的性质。腐蚀和膨胀是两个基本的形态学运算符。然后它的变体形式如开运算，闭运算，梯度等也发挥作用。

- 腐蚀

腐蚀的基本思想就像土壤侵蚀一样，它会腐蚀前景物体的边界（总是试图保持前景为白色）。它是如何做到的呢？卷积核在图像中滑动（如在2D卷积中），只有当卷积核下的所有像素都是1时，原始图像中的像素（1或0）才会被认为是1，否则它会被腐蚀（变为零）。

所以腐蚀作用后，边界附近的所有像素都将被丢弃，具体取决于卷积核的大小。因此，前景对象的厚度或大小减小，或者图像中的白色区域减小。它有助于消除小的白噪声（正如我们在色彩空间章节中看到的那样），或者分离两个连接的对象等。
```cpp
/*
void cv::erode(
    InputArray src,          // 输入图像
    OutputArray dst,         // 输出图像
    InputArray kernel,       // 结构元素
    Point anchor = Point(-1,-1), // 锚点位置，默认是内核中心
    int iterations = 1,      // 迭代次数
    int borderType = BORDER_CONSTANT, // 边界模式
    const Scalar& borderValue = Scalar() // 边界填充值
);
*/

cv::Mat src, dst;
cv::Mat kernel = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(5, 5));
cv::erode(src, dst, kernel, cv::Point(-1, -1), 1);
```

- 膨胀

它恰好与腐蚀相反。这里，如果卷积核下的像素至少一个像素为“1”，则像素元素为“1”。因此它增加了图像中的白色区域或前景对象的大小。通常，在去除噪音的情况下，侵蚀之后是扩张。因为，侵蚀会消除白噪声，但它也会缩小我们的物体,所以我们扩大它。由于噪音消失了，它们不会再回来，但我们的物体区域会增加。它也可用于连接对象的破碎部分。
```cpp
void cv::dilate(
    InputArray src,          // 输入图像
    OutputArray dst,         // 输出图像
    InputArray kernel,       // 结构元素
    Point anchor = Point(-1,-1), // 锚点位置，默认是内核中心
    int iterations = 1,      // 迭代次数
    int borderType = BORDER_CONSTANT, // 边界模式
    const Scalar& borderValue = Scalar() // 边界填充值
);
```

- 开运算

开运算是先腐蚀后膨胀的合成步骤。如上所述，它有助于消除噪音。
```cpp
/*
void cv::morphologyEx(
    InputArray src,          // 输入图像
    OutputArray dst,         // 输出图像
    int op,                  // 形态学操作类型，开运算为 cv::MORPH_OPEN
    InputArray kernel,       // 结构元素
    Point anchor = Point(-1,-1), // 锚点位置，默认是内核中心
    int iterations = 1,      // 迭代次数
    int borderType = BORDER_CONSTANT, // 边界模式
    const Scalar& borderValue = Scalar() // 边界填充值
);
*/

cv::morphologyEx(src, dst, cv::MORPH_OPEN, kernel, cv::Point(-1, -1), 1);
```

- 闭运算

闭运算与开运算相反，他是先膨胀后腐蚀的操作。它可用于过滤前景对象内的小孔或对象上的小黑点。
```cpp
cv::morphologyEx(src, dst, cv::MORPH_CLOSE, kernel, cv::Point(-1, -1), 1);
```

- 梯度

梯度运算是膨胀和腐蚀之间的差异，通常用于边缘检测。
```cpp
cv::morphologyEx(src, dst, cv::MORPH_GRADIENT, kernel, cv::Point(-1, -1), 1);
```

- 顶帽

顶帽运算是原图像与开运算结果之间的差异，主要用于提取图像中的小的明亮区域。
```cpp
cv::morphologyEx(src, dst, cv::MORPH_TOPHAT, kernel, cv::Point(-1, -1), 1);
```

- 黑帽

黑帽运算是闭运算结果与原图像之间的差异，主要用于提取图像中的小的暗区域。
```cpp
cv::morphologyEx(src, dst, cv::MORPH_BLACKHAT, kernel, cv::Point(-1, -1), 1);
```

- 结构元素

`cv::getStructuringElement()`结构元素是一个小矩阵，用于定义在形态学操作中要应用的卷积区域。它可以是不同的形状，如矩形、椭圆形或交叉形。

## 图形梯度

图形梯度（Gradient）是图像处理中一种用于检测图像边缘和变化的方法。它描述了图像中亮度变化的方向和程度。梯度信息可以帮助提取图像中的边缘和细节，从而用于图像分析、特征提取等任务。

- Sobel算子

Sobel算子是高斯联合平滑加微分运算，因此它更能抵抗噪声。你可以指定要采用的导数的方向，垂直或水平（yorder和xorder），你还可以通过参数ksize指定卷积核的大小
```cpp
/*
void cv::Sobel(
    InputArray src,          // 输入图像
    OutputArray dst,         // 输出图像
    int ddepth,              // 输出图像的深度
    int dx,                  // x 方向的阶数
    int dy,                  // y 方向的阶数
    int ksize = 3,           // Sobel 核的大小
    double scale = 1,        // 缩放因子
    double delta = 0,        // 加到结果上的值
    int borderType = BORDER_DEFAULT // 边界模式
);
*/

cv::Mat src, grad_x, grad_y;
cv::Mat grad;
int ddepth = CV_16S;
cv::Sobel(src, grad_x, ddepth, 1, 0, 3); // 计算 x 方向的梯度
cv::Sobel(src, grad_y, ddepth, 0, 1, 3); // 计算 y 方向的梯度
cv::convertScaleAbs(grad_x, grad_x); // 转换为 8 位图像
cv::convertScaleAbs(grad_y, grad_y); // 转换为 8 位图像
cv::addWeighted(grad_x, 0.5, grad_y, 0.5, 0, grad); // 合并 x 和 y 方向的梯度
```

- Scharr算子

Scharr 算子是一种改进版的 Sobel 算子，提供了更高的边缘检测效果。
```cpp
/*
void cv::Scharr(
    InputArray src,          // 输入图像
    OutputArray dst,         // 输出图像
    int ddepth,              // 输出图像的深度
    int dx,                  // x 方向的阶数
    int dy,                  // y 方向的阶数
    double scale = 1,        // 缩放因子
    double delta = 0,        // 加到结果上的值
    int borderType = BORDER_DEFAULT // 边界模式
);
*/

cv::Mat src, grad_x, grad_y;
cv::Mat grad;
int ddepth = CV_16S;
cv::Scharr(src, grad_x, ddepth, 1, 0); // 计算 x 方向的梯度
cv::Scharr(src, grad_y, ddepth, 0, 1); // 计算 y 方向的梯度
cv::convertScaleAbs(grad_x, grad_x); // 转换为 8 位图像
cv::convertScaleAbs(grad_y, grad_y); // 转换为 8 位图像
cv::addWeighted(grad_x, 0.5, grad_y, 0.5, 0, grad); // 合并 x 和 y 方向的梯度
```

- Laplacian算子

Laplacian 算子是图像处理中一种用于检测图像中的边缘和细节的二阶导数算子。它通过计算图像中像素的二阶导数来检测边缘和变化区域。
```cpp
/*
void cv::Laplacian(
    InputArray src,          // 输入图像
    OutputArray dst,         // 输出图像
    int ddepth,              // 输出图像的深度
    int ksize = 3,           // 内核大小
    double scale = 1,        // 缩放因子
    double delta = 0,        // 加到结果上的值
    int borderType = BORDER_DEFAULT // 边界模式
);
*/

cv::Mat src, dst;
cv::Mat src_gray;
cv::cvtColor(src, src_gray, cv::COLOR_BGR2GRAY); // 将图像转换为灰度图像
cv::Laplacian(src_gray, dst, CV_16S, 3);
cv::Mat abs_dst;
cv::convertScaleAbs(dst, abs_dst); // 将结果转换为 8 位图像
```

## Canny边缘检测

Canny 边缘检测是一种多阶段的边缘检测算法。它由 John F. Canny 在 1986 年提出，具有良好的边缘检测性能，能够有效地检测到图像中的边缘并且减少噪声影响。Canny边缘检测算法通常包含以下几个主要步骤：
1. 降噪

由于边缘检测易受图像中的噪声影响，因此第一步是使用5x5高斯滤波器去除图像中的噪声。

2. 梯度计算

计算图像中每个像素的梯度幅值和方向。通常使用 Sobel 算子来计算水平和垂直方向上的梯度。

3. 非极大值抑制

对梯度幅值图像进行处理，仅保留边缘的局部最大值。即在梯度方向上，只有幅值最大的像素被保留下来，其他像素被抑制。

4. 双阈值检测

使用两个阈值来确定边缘像素：高阈值和低阈值。边缘强度高于高阈值的像素被认为是强边缘，低于低阈值的像素被认为是非边缘，介于两个阈值之间的像素被认为是弱边缘。

5. 边缘链接

将弱边缘连接到强边缘。弱边缘仅在连接到强边缘的情况下才被保留，以确保边缘的连贯性。

```cpp
void cv::Canny(
    InputArray image,      // 输入图像，通常是灰度图像
    OutputArray edges,    // 输出图像，边缘像素为 255，其他为 0
    double threshold1,    // 第一阈值（低阈值）
    double threshold2,    // 第二阈值（高阈值）
    int apertureSize = 3, // Sobel 算子的内核大小，默认为 3
    bool L2gradient = false // 是否使用 L2 范数来计算梯度幅值，默认为 false
);
```

## 轮廓

轮廓可以简单地解释为连接所有具有相同的颜色或强度的连续点（沿着边界）的曲线。轮廓是形状分析和物体检测和识别的很有用的工具。在OpenCV中，找到轮廓就像从黑色背景中找到白色物体。所以请记住，要找到的对象应该是白色，背景应该是黑色。为了更好的准确性，使用二进制图像，因此，在找到轮廓之前，应用阈值或canny边缘检测。
```cpp
/*
void cv::findContours(
    InputOutputArray image,    // 输入的二值图像，图像在检测过程中会被修改
    OutputArrayOfArrays contours, // 输出的轮廓，每个轮廓由一系列点组成
    OutputArray hierarchy,     // 可选的输出参数，轮廓的层次结构
    int mode,                  // 检测轮廓的模式
    int method,                // 轮廓的近似方法
    Point offset = Point()     // 轮廓点的偏移量
);
*/

// 检测轮廓
std::vector<std::vector<cv::Point>> contours;
std::vector<cv::Vec4i> hierarchy;
cv::findContours(binary, contours, hierarchy, cv::RETR_TREE, cv::CHAIN_APPROX_SIMPLE);

// 绘制轮廓
cv::Mat drawing = cv::Mat::zeros(binary.size(), CV_8UC3);
for (size_t i = 0; i < contours.size(); i++) {
    cv::Scalar color = cv::Scalar(0, 255, 0); // 绿色
    cv::drawContours(drawing, contours, static_cast<int>(i), color, 2, 8, hierarchy, 0);
}
```

- 轮廓特征

轮廓特征是描述图像中检测到的轮廓的形状、大小、位置等属性的各种度量和描述。
```cpp
// 轮廓面积
double area = cv::contourArea(contours[i]);

// 轮廓周长
double perimeter = cv::arcLength(contours[i], true);

// 边界矩形
cv::Rect boundingBox = cv::boundingRect(contours[i]);   // 轴对齐矩形
cv::RotatedRect minRect = cv::minAreaRect(contours[i]); // 最小旋转矩形

// 最小外接圆
cv::Point2f center;
float radius;
cv::minEnclosingCircle(contours[i], center, radius);

// 最小外接椭圆（需要至少5个点）
if (contours[i].size() >= 5) {
    cv::RotatedRect minEllipse = cv::fitEllipse(contours[i]);
}

// 凸包
std::vector<cv::Point> hull;
cv::convexHull(contours[i], hull);
```

- 层次结构

轮廓层次结构是描述图像中不同轮廓之间关系的方式，特别是在包含嵌套轮廓的场景中，如带有孔洞的对象、嵌套的形状等。OpenCV在检测轮廓时，可以使用`cv::findContours`函数来构建轮廓的层次结构，并返回这些轮廓之间的父子关系，OpenCV将它表示为四个值的数组：`[Next，Previous，First_Child，Parent]`。同一个轮廓，采用不同的检索模式（retrieval mode），四个值可能不同。

## 直方图

直方图是什么？你可以将直方图视为图形或绘图，它可以让你全面了解图像的强度分布。它是在X轴上具有像素值（范围从0到255，并非总是）的图和在Y轴上的图像中的对应像素数。通过查看图像的直方图，你可以直观了解该图像的对比度，亮度，强度分布等。

考虑一个像素值仅限于某些特定值范围的图像。 例如，较亮的图像将所有像素限制为高值。 但是，良好的图像将具有来自图像的所有区域的像素。 所以你需要将这个直方图拉伸到两端，这就是直方图均衡所做的，通常可以改善图像的对比度。这种方法对于背景和前景都太亮或者太暗的图像非常有用，这种方法尤其是可以带来X光图像中更好的骨骼结构显示以及曝光过度或者曝光不足照片中更好的细节。这种方法的一个主要优势是它是一个相当直观的技术并且是可逆操作，如果已知均衡化函数，那么就可以恢复原始的直方图，并且计算量也不大。这种方法的一个缺点是它对处理的数据不加选择，它可能会增加背景噪声的对比度并且降低有用信号的对比度。

## 霍夫变换

霍夫变换（Hough Transform）是一种用于检测图像中几何形状（如直线、圆等）的技术。它特别适用于检测具有特定形状的对象，即使这些对象存在噪声或部分缺失。霍夫变换将形状检测问题转化为参数空间中的峰值检测问题。

## 分水岭算法

任何灰度图像都可以看作是地形表面，其中高强度表示峰和丘陵，而低强度表示山谷。你开始用不同颜色的水（标签）填充每个孤立的山谷（局部最小值）。随着水的上升，取决于附近的峰值（梯度），来自不同山谷的水，明显具有不同的颜色将开始融合。为避免这种情况，你需要在水合并的位置建立障碍。你继续填补水和建筑障碍的工作，直到所有的山峰都在水下。然后，你创建的障碍将为你提供分割结果。

但是，由于噪声或图像中的任何其他不规则性，此方法会为你提供过度调整结果。因此，OpenCV实现了一个基于标记的分水岭算法，你可以在其中指定哪些是要合并的谷点，哪些是不合并的。它是一种交互式图像分割。我们所做的是为我们所知道的对象提供不同的标签。用一种颜色（或强度）标记我们确定为前景或对象的区域，用另一种颜色标记我们确定为背景或非对象的区域，最后标记我们不确定的区域，用0标记它。这是我们的标记。然后应用分水岭算法。然后我们的标记将使用我们给出的标签进行更新，对象的边界将具有-1的值。
