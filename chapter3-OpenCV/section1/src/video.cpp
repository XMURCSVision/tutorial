#include <stdio.h>

#include <opencv2/opencv.hpp>

int main(int argc, char** argv) {
  if (argc != 3) {
    printf("usage: video <src_path> <dst_path>\n");
    return -1;
  }

  // 打开原始视频
  cv::VideoCapture cap(argv[1]);
  if (!cap.isOpened()) {
    printf("Error: cannot open video file\n");
    return -1;
  }

  // 获取原视频帧率和帧大小
  double fps = cap.get(cv::CAP_PROP_FPS);
  int frameWidth = static_cast<int>(cap.get(cv::CAP_PROP_FRAME_WIDTH));
  int frameHeight = static_cast<int>(cap.get(cv::CAP_PROP_FRAME_HEIGHT));

  // 定义输出视频文件的名称和编码格式
  cv::VideoWriter writer;
  int codec = cv::VideoWriter::fourcc('a', 'v', 'c', '1');  // H.264 编码格式
  writer.open(argv[2], codec, fps, cv::Size(frameWidth, frameHeight), true);

  if (!writer.isOpened()) {
    printf("Error: cannot open video writer\n");
    return -1;
  }

  std::vector<cv::Mat> frames;
  cv::Mat frame;
  while (true) {
    cap >> frame;
    if (frame.empty()) {
      break;
    }
    frames.push_back(frame.clone());
  }

  // 倒序写入视频
  for (auto it = frames.rbegin(); it != frames.rend(); ++it) {
    writer.write(*it);
  }

  // 释放资源
  cap.release();
  writer.release();

  printf("Video saved to %s\n", argv[2]);
  return 0;
}