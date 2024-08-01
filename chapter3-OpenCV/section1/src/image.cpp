#include <stdio.h>

#include <opencv2/opencv.hpp>

using namespace cv;

int main(int argc, char** argv) {
  if (argc != 3) {
    printf("usage: image <src_path> <dst_path>\n");
    return -1;
  }

  Mat image;
  image = imread(argv[1], IMREAD_GRAYSCALE);  // 读取图像

  if (!image.data) {
    printf("No image data \n");
    return -1;
  }

  namedWindow("Display Image", WINDOW_NORMAL);  // 创建窗口
  imshow("Display Image", image);               // 显示图像

  int keyboard = waitKey(0);
  if (keyboard == 27) {
    std::cout << "quit" << std::endl;
    destroyAllWindows();
  } else if (keyboard == 's') {
    std::cout << "save image to " << argv[2] << std::endl;
    imwrite(argv[2], image);
    destroyAllWindows();
  } else {
    destroyAllWindows();
  }

  return 0;
}