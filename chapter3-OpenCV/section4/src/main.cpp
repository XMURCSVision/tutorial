#include <algorithm>
#include <filesystem>
#include <map>
#include <memory>
#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>
#include <string>
#include <vector>

#include "armor.hpp"
#include "main.hpp"

int main(int argc, char* argv[]) {
  if (argc < 2) {
    std::cerr << "Usage: " << argv[0] << " <video_path>" << std::endl;
    return 1;
  }

  cap.open(argv[1]);
  if (!cap.isOpened()) {
    std::cerr << "Error: cannot open video file" << std::endl;
    return 1;
  }

  // Detector
  detector_ = initDetector();

  debug_ = true;  // TODO：从yaml文件读取

  cv::Mat frame;
  while (cap.read(frame)) {
    auto armors = detectArmors(frame);

    cv::waitKey(0);
    cv::imshow("frame", frame);
  }

  cap.release();
  cv::destroyAllWindows();

  return 0;
}

std::unique_ptr<Detector> initDetector() {
  int binary_thres = 160;
  auto detect_color = 0;
  Detector::LightParams l_params = {
      .min_ratio = 0.1, .max_ratio = 0.4, .max_angle = 40.0};

  Detector::ArmorParams a_params = {.min_light_ratio = 0.7,
                                    .min_small_center_distance = 0.8,
                                    .max_small_center_distance = 3.2,
                                    .min_large_center_distance = 3.2,
                                    .max_large_center_distance = 5.5,
                                    .max_angle = 35.0};

  auto detector = std::make_unique<Detector>(binary_thres, detect_color,
                                             l_params, a_params);

  // Init classifier
  auto model_path = "../model/mlp.onnx";
  auto label_path = "../model/label.txt";
  double threshold = 0.7;
  std::vector<std::string> ignore_classes =
      std::vector<std::string>{"negative"};
  detector->classifier = std::make_unique<NumberClassifier>(
      model_path, label_path, threshold, ignore_classes);

  return detector;
}

std::vector<Armor> detectArmors(const cv::Mat& img) {
  // TODO:update params

  auto armors = detector_->detect(img);

  if (debug_) {
    // TODO:drawing
    for (auto armor : armors) {
      std::cout << "debug: " << armor.classfication_result << std::endl;
    }

    if (!armors.empty()) {
      auto all_num_img = detector_->getAllNumbersImage();
      cv::imshow("all_num_img", all_num_img);
    }

    cv::Mat result_img = img.clone();
    detector_->drawResults(result_img);
    cv::imshow("result_img", result_img);
  }

  return armors;
}