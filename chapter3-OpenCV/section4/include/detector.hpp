#ifndef DETECTOR_HPP
#define DETECTOR_HPP

// OpenCV
#include <opencv2/opencv.hpp>

// STL
#include <vector>

#include "armor.hpp"
#include "number_classifier.hpp"

class Detector {
 public:
  struct LightParams {
    // width / height
    double min_ratio;
    double max_ratio;
    // vertical angle
    double max_angle;
  };

  struct ArmorParams {
    double min_light_ratio;
    // light pairs distance
    double min_small_center_distance;
    double max_small_center_distance;
    double min_large_center_distance;
    double max_large_center_distance;
    // horizontal angle
    double max_angle;
  };

  Detector(const int &bin_thres, const int &color, const LightParams &l,
           const ArmorParams &a);

  std::vector<Armor> detect(const cv::Mat &input);

  cv::Mat preprocessImage(const cv::Mat &input);
  std::vector<Light> findLights(const cv::Mat &rbg_img,
                                const cv::Mat &binary_img);
  std::vector<Armor> matchLights(const std::vector<Light> &lights);

  cv::Mat getAllNumbersImage();
  void drawResults(cv::Mat &img);

  int binary_thres;
  int detect_color;
  LightParams l;
  ArmorParams a;

  std::unique_ptr<NumberClassifier> classifier;

  cv::Mat binary_img;

 private:
  bool isLight(const Light &possible_light);
  bool containLight(const Light &light_1, const Light &light_2,
                    const std::vector<Light> &lights);
  ArmorType isArmor(const Light &light_1, const Light &light_2);

  std::vector<Light> lights_;
  std::vector<Armor> armors_;
};

#endif  // DETECTOR_HPP