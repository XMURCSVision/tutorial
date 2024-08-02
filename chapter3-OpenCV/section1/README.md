# OpenCV中的Gui特性

## 图像入门

- 读取图像

`cv::imread()`函数用于读取图像，该函数包含两个参数，第一个参数为图像路径，第二个参数默认为`cv::IMREAD_COLOR`，表示以彩色模式加载图像（BGR），其他参数如`cv::IMREAD_GRAYSCALE`以灰度模式加载图像、`cv::IMREAD_UNCHANGED`以alpha通道模式加载图像。
```cpp
// cv::Mat cv::imread(const String &filename, int flags=cv::IMREAD_COLOR)
cv::Mat src = cv::imread("filename");
```

- 显示图像

`cv::imshow()`函数被用于在窗口中显示图像，窗口会自动适应图像大小。其中，函数的第一个参数是窗口的名称，是字符串类型。第二个参数是要加载的图像。你可以显示多个图像窗口，但是每个窗口名称必须不同。
```cpp
// void cv::imshow(const String &winname, InputArray mat);
cv::imshow("display", src);
cv::waitKey(0);
cv::destroyAllWindows();
```
`cv::waitKey()`是一个键盘事件函数，它的参数以毫秒为单位，该函数在毫秒的时间内去等待键盘事件，如果时间之内有键盘事件触发则程序继续，如果函数参数设置为0，则无限时间的等待键盘事件触发。它也可以设置为检测指定按键的触发。

`cv::destroyAllWindow()`函数用于关闭我们所创建的所有显示图像的窗口，如果想要关闭特定的窗口，请使用`cv::destroyWindow()`函数，把要关闭的窗口名称作为参数。

如果你想要手动调整窗口大小，那么你需要先创建一个窗口，再加载图像。使用`cv::nameWindow()`函数自行调整窗口大小。函数默认参数是`cv::WINDOW_AUTOSIZE`。你可以使用`cv::WINDOW_NORMAL`函数，以自行调整窗口大小。
```cpp
cv::namedWindow("image", cv::WINDOW_NORMAL);
cv::imshow("image", src);
cv::waitKey(0);
cv::destroyAllWindows();
```

- 保存图像

`cv::imwrite()`函数用于保存图像。其中第一个参数是保存为的图片名，第二个参数为待保存图像。
```cpp
// bool cv::imwrite(const String &filename, InputArray img, const std::vector<int> &params=std::vector<int>())
cv::imwrite("test.png", src);
```
## 视频入门
- 用摄像头捕获视频

如果要捕获视频，首先要做的是创建一个`cv::VideoCapture`对象，它的参数可以是设备索引或者是视频文件的名称。设备索引就是指设备所对应的设备号，当只连接一个摄像头，参数只需传递0（或-1） 。你可以传递参数1来选择你连接的第二个摄像头等等。接下来，你需要逐帧捕获并显示并不要忘记关闭捕获。
```cpp
cv::VideoCapture cap;
cap.open(0);  // cap.open("filename");

if(!cap.isOpened()){
    std::cout << "Open failed" << std::endl;
    return -1;
}

cv::Mat frame;
while(cap.read(frame)){
    // 图像操作
}

// 释放
cap.release();
cv::destroyAllWindows();
```

- 播放视频文件

与从相机捕获视频原理相同，只需将设备索引更改为视频文件的名字。同时在显示帧时，请给`cv.waitKey()`函数传递适当的时间参数。如果它太小，视频将非常快，如果它太高，视频将会很慢。在正常情况下，25毫秒就可以了。

- 保存视频

保存图片很简单，但是对于视频， 相对繁琐很多。 首先创建一个`cv::VideoWriter`对象，我们应该指定输出文件名（例如：output.avi），然后我们应该指定FourCC代码并传递每秒帧数（fps）和帧大小。最后一个是isColor标志，如果是True，则每一帧是彩色图像，否则每一帧是灰度图像。 FourCC是用于指定视频编解码器的4字节代码。
```cpp
// 定义视频文件的名称和编码格式
std::string filename = "output_video.avi";
int codec = cv::VideoWriter::fourcc('M', 'J', 'P', 'G'); // MPEG-4编码
double fps = 30.0; // 每秒30帧

// 创建 VideoWriter 对象
cv::VideoWriter writer;
writer.open(filename, codec, fps, cv::Size(640, 480), true);

// 检查 VideoWriter 是否成功打开
if (!writer.isOpened()) {
    std::cerr << "无法打开视频文件进行写入" << std::endl;
    return -1;
}

// 创建一个用于写入的视频帧
cv::Mat frame = cv::Mat::zeros(480, 640, CV_8UC3);

// 主循环：生成一些简单的动画
for (int i = 0; i < 100; ++i) {
    // 填充帧数据 (例如，绘制一个移动的圆)
    frame.setTo(cv::Scalar(0, 0, 0)); // 清空帧
    cv::circle(frame, cv::Point(320, 240), i, cv::Scalar(255, 0, 0), -1);

    cv::imshow("Frame", frame);
    if (cv::waitKey(30) >= 0) break; // 按任意键退出

    // 将帧写入视频文件
    writer.write(frame);
}

writer.release();
cv::destroyAllWindows();
```

## 绘图

- 线段
```cpp
void cv::line(
    InputOutputArray img, Point pt1, Point pt2,
    const Scalar &color, int thickness = 1, int lineType = LINE_8, int shift = 0
);	
```

- 矩形
```cpp
void cv::rectangle(
    InputOutputArray img, Point pt1, Point pt2,
    const Scalar &color, int thickness = 1, int lineType = LINE_8, int shift = 0
);
```

- 圆形
```cpp
void cv::circle(
    InputOutputArray img, Point center, int radius,
    const Scalar &color, int thickness = 1, int lineType = LINE_8, int shift = 0
);
```

- 椭圆
```cpp
void cv::ellipse(
    InputOutputArray img, Point center, Size axes, double angle,
    double startAngle, double endAngle,
    const Scalar &color, int thickness = 1, int lineType = LINE_8, int shift = 0
);
```

- 多边形
```cpp
void cv::polylines(
    InputOutputArray img, const std::vector<std::vector<Point>> &pts,
    bool isClosed, const Scalar &color, int thickness = 1,
    int lineType = LINE_8, int shift = 0
);	
```

- 文字
```cpp
void cv::putText(
    InputOutputArray img, const std::string &text, Point org,
    int fontFace, double fontScale, const Scalar &color,
    int thickness = 1, int lineType = LINE_8, bool bottomLeftOrigin = false
);
```

## Task
- [ ] 读取一张彩色图像并显示为灰度图像，图像窗口可以手动调整，按`s`键保存图像并退出，按`ESC`键直接退出且不保存。
- [ ] 读取一个视频并保存为倒放的视频。
- [ ] 在图像上绘制线段、矩形、圆形、椭圆、多边形、文字。