#include <stdio.h>

#include <opencv2/opencv.hpp>

int main(int argc, char *argv[]) {
  if (argc != 2) {
    printf("Usage: mask <src_path>\n");
    return -1;
  }

  cv::Mat src = cv::imread(argv[1], cv::IMREAD_COLOR);
  cv::imshow("src", src);

  cv::Mat mask = cv::Mat::zeros(src.size(), CV_8UC1);
  cv::rectangle(mask, cv::Rect(100, 100, 200, 200), cv::Scalar(255), -1);
  cv::imshow("mask", mask);

  cv::Mat dst;
  cv::bitwise_and(src, src, dst, mask);
  cv::imshow("dst", dst);

  cv::waitKey(0);
  cv::destroyAllWindows();

  return 0;
}