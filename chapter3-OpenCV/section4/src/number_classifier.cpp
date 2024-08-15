// OpenCV
#include <opencv2/core.hpp>
#include <opencv2/core/mat.hpp>
#include <opencv2/core/types.hpp>
#include <opencv2/dnn.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/opencv.hpp>

// STL
#include <algorithm>
#include <cstddef>
#include <fstream>
#include <map>
#include <string>
#include <vector>

#include "armor.hpp"
#include "number_classifier.hpp"

NumberClassifier::NumberClassifier(
    const std::string& model_path, const std::string& label_path,
    const double thre, const std::vector<std::string>& ignore_classes)
    : threshold(thre), ignore_classes_(ignore_classes) {
  net_ = cv::dnn::readNetFromONNX(model_path);

  std::ifstream label_file(label_path);
  std::string line;
  while (std::getline(label_file, line)) {
    class_names_.push_back(line);
  }
}

void NumberClassifier::extractNumbers(const cv::Mat& src,
                                      std::vector<Armor>& armors) {
  // Light length in image
  const int light_length = 12;
  // Image size after warp
  const int warp_height = 28;
  const int small_armor_width = 32;
  const int large_armor_width = 54;
  // Number ROI size
  const cv::Size roi_size(20, 28);

  for (auto &armor : armors) {
    // TODO：提取出装甲板两个灯条间的ROI

    armor.number_img = number_image;
  }
}

void NumberClassifier::classify(std::vector<Armor>& armors) {
  for (auto& armor : armors) {
    cv::Mat image = armor.number_img.clone();

    // Normalize
    image = image / 255.0;

    // Create blob from image
    cv::Mat blob;
    cv::dnn::blobFromImage(image, blob);

    // Set the input blob for the neural network
    net_.setInput(blob);
    // Forward pass the image blob through the model
    cv::Mat outputs = net_.forward();

    // Do softmax
    float max_prob =
        *std::max_element(outputs.begin<float>(), outputs.end<float>());
    cv::Mat softmax_prob;
    cv::exp(outputs - max_prob, softmax_prob);
    float sum = static_cast<float>(cv::sum(softmax_prob)[0]);
    softmax_prob /= sum;

    double confidence;
    cv::Point class_id_point;
    minMaxLoc(softmax_prob.reshape(1, 1), nullptr, &confidence, nullptr,
              &class_id_point);
    int label_id = class_id_point.x;

    armor.confidence = confidence;
    armor.number = class_names_[label_id];

    std::stringstream result_ss;
    result_ss << armor.number << ": " << std::fixed << std::setprecision(1)
              << armor.confidence * 100.0 << "%";
    armor.classfication_result = result_ss.str();
  }

  armors.erase(
      std::remove_if(armors.begin(), armors.end(),
                     [this](const Armor& armor) {
                       if (armor.confidence < threshold) {
                         return true;
                       }

                       for (const auto& ignore_class : ignore_classes_) {
                         if (armor.number == ignore_class) {
                           return true;
                         }
                       }

                       bool mismatch_armor_type = false;
                       if (armor.type == ArmorType::LARGE) {
                         mismatch_armor_type = armor.number == "outpost" ||
                                               armor.number == "2" ||
                                               armor.number == "guard";
                       } else if (armor.type == ArmorType::SMALL) {
                         mismatch_armor_type =
                             armor.number == "1" || armor.number == "base";
                       }
                       return mismatch_armor_type;
                     }),
      armors.end());
}