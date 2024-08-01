#include <stdio.h>

#include <opencv2/opencv.hpp>

int main(int argc, char** argv) {
  if (argc != 3) {
    printf("usage: draw <src_path> <dst_path>\n");
    return -1;
  }

  cv::Mat src = cv::imread(argv[1]);

  // 绘制线段
  cv::line(src, cv::Point(0, 0), cv::Point(100, 100), cv::Scalar(0, 0, 255), 2);

  // 绘制矩形
  cv::rectangle(src, cv::Point(100, 100), cv::Point(200, 200),
                cv::Scalar(0, 255, 0), 2);

  // 绘制圆形
  cv::circle(src, cv::Point(300, 300), 50, cv::Scalar(255, 0, 0), 2);

  // 绘制椭圆
  cv::ellipse(src, cv::Point(400, 400), cv::Size(100, 50), 45, 0, 360,
              cv::Scalar(255, 255, 0), 2);

  // 绘制多边形
  std::vector<cv::Point> points;
  points.push_back(cv::Point(100, 400));
  points.push_back(cv::Point(200, 400));
  points.push_back(cv::Point(200, 500));
  points.push_back(cv::Point(100, 500));
  cv::polylines(src, points, true, cv::Scalar(0, 255, 255), 2);

  // 绘制文字
  cv::putText(src, "Hello OpenCV", cv::Point(100, 100),
              cv::FONT_HERSHEY_SIMPLEX, 1, cv::Scalar(255, 255, 255), 2);

  cv::imshow("Draw", src);
  cv::waitKey(0);

  cv::imwrite(argv[2], src);

  cv::destroyAllWindows();
}