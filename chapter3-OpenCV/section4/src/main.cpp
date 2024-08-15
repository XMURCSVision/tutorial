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

  // get parameters
  config = YAML::LoadFile(std::string(PROJECT_SOURCE_DIR) +
                          "/config/detect_params.yaml");

  // Detector
  detector_ = initDetector();

  debug_ = config["debug"].as<bool>();

  cv::Mat frame;
  while (cap.read(frame)) {
    cv::resize(frame, frame, cv::Size(), 0.5, 0.5);

    auto armors = detectArmors(frame);

    cv::waitKey(config["waitkey"].as<int>());
  }

  cap.release();
  cv::destroyAllWindows();

  return 0;
}

std::unique_ptr<Detector> initDetector() {
  int binary_thres = config["binary_thres"].as<int>();
  auto detect_color = config["detect_color"].as<int>();
  Detector::LightParams l_params = {
      .min_ratio = config["light"][0]["min_ratio"].as<double>(),
      .max_ratio = config["light"][1]["max_ratio"].as<double>(),
      .max_angle = config["light"][2]["max_angle"].as<double>()};

  Detector::ArmorParams a_params = {
      .min_light_ratio = config["armor"][0]["min_light_ratio"].as<double>(),
      .min_small_center_distance =
          config["armor"][1]["min_small_center_distance"].as<double>(),
      .max_small_center_distance =
          config["armor"][2]["max_small_center_distance"].as<double>(),
      .min_large_center_distance =
          config["armor"][3]["min_large_center_distance"].as<double>(),
      .max_large_center_distance =
          config["armor"][4]["max_large_center_distance"].as<double>(),
      .max_angle = config["armor"][5]["max_angle"].as<double>()};

  auto detector = std::make_unique<Detector>(binary_thres, detect_color,
                                             l_params, a_params);

  // Init classifier
  auto model_path =
      std::string(PROJECT_SOURCE_DIR) + config["model_path"].as<std::string>();
  auto label_path =
      std::string(PROJECT_SOURCE_DIR) + config["label_path"].as<std::string>();
  double threshold = config["classifier_threshold"].as<double>();
  std::vector<std::string> ignore_classes =
      config["ignore_classes"].as<std::vector<std::string>>();
  ;
  detector->classifier = std::make_unique<NumberClassifier>(
      model_path, label_path, threshold, ignore_classes);

  return detector;
}

std::vector<Armor> detectArmors(const cv::Mat& img) {
  auto armors = detector_->detect(img);

  if (debug_) {
    if (!armors.empty()) {
      auto all_num_img = detector_->getAllNumbersImage();
      cv::resize(all_num_img, all_num_img, cv::Size(120, 168));
      cv::imshow("all_num_img", all_num_img);
    }

    cv::Mat result_img = img.clone();
    detector_->drawResults(result_img);
    cv::imshow("result_img", result_img);
  }

  return armors;
}