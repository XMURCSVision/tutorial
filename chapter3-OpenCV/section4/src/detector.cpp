// OpenCV
#include <opencv2/core.hpp>
#include <opencv2/core/base.hpp>
#include <opencv2/core/mat.hpp>
#include <opencv2/core/types.hpp>
#include <opencv2/imgproc.hpp>

// STD
#include <algorithm>
#include <cmath>
#include <vector>

#include "detector.hpp"

Detector::Detector(const int& bin_thres, const int& color, const LightParams& l,
                   const ArmorParams& a)
    : binary_thres(bin_thres), detect_color(color), l(l), a(a) {}

std::vector<Armor> Detector::detect(const cv::Mat& input) {
  binary_img = preprocessImage(input);
  lights_ = findLights(input, binary_img);
  armors_ = matchLights(lights_);

  if (!armors_.empty()) {
    classifier->extractNumbers(input, armors_);
    classifier->classify(armors_);
  }

  return armors_;
}

cv::Mat Detector::preprocessImage(const cv::Mat &rgb_img) {
  //TODO: 请在这里对图像进行预处理操作，注意输入输出图像格式
  return binary_img;
}

std::vector<Light> Detector::findLights(const cv::Mat &rbg_img,
                                        const cv::Mat &binary_img) {
  using std::vector;
  // TODO：请在这里提取轮廓

  vector<Light> lights;

  for (const auto& contour : contours) {
    if (contour.size() < 5) continue;

    auto r_rect = cv::minAreaRect(contour);
    auto light = Light(r_rect);

    if (isLight(light)) {
      auto rect = light.boundingRect();
      if (  // Avoid assertion failed
          0 <= rect.x && 0 <= rect.width &&
          rect.x + rect.width <= rbg_img.cols && 0 <= rect.y &&
          0 <= rect.height && rect.y + rect.height <= rbg_img.rows) {
        int sum_r = 0, sum_b = 0;
        auto roi = rbg_img(rect);
        // Iterate through the ROI
        for (int i = 0; i < roi.rows; i++) {
          for (int j = 0; j < roi.cols; j++) {
            if (cv::pointPolygonTest(
                    contour, cv::Point2f(j + rect.x, i + rect.y), false) >= 0) {
              // if point is inside contour
              sum_r += roi.at<cv::Vec3b>(i, j)[0];
              sum_b += roi.at<cv::Vec3b>(i, j)[2];
            }
          }
        }
        // Sum of red pixels > sum of blue pixels ?
        light.color = sum_r > sum_b ? RED : BLUE;
        lights.emplace_back(light);
      }
    }
  }

  return lights;
}

bool Detector::isLight(const Light &light) {
  // TODO：请在这里完成对灯条的判断
  // 提示，你可以从detector类中的lightparams中的参数入手
}

std::vector<Armor> Detector::matchLights(const std::vector<Light>& lights) {
  std::vector<Armor> armors;

  // Loop all the pairing of lights
  for (auto light_1 = lights.begin(); light_1 != lights.end(); light_1++) {
    for (auto light_2 = light_1 + 1; light_2 != lights.end(); light_2++) {
      if (light_1->color != detect_color || light_2->color != detect_color)
        continue;

      if (containLight(*light_1, *light_2, lights)) {
        continue;
      }

      auto type = isArmor(*light_1, *light_2);
      if (type != ArmorType::INVALID) {
        auto armor = Armor(*light_1, *light_2);
        armor.type = type;
        armors.emplace_back(armor);
      }
    }
  }

  return armors;
}

// Check if there is another light in the boundingRect formed by the 2 lights
bool Detector::containLight(const Light& light_1, const Light& light_2,
                            const std::vector<Light>& lights) {
  auto points = std::vector<cv::Point2f>{light_1.top, light_1.bottom,
                                         light_2.top, light_2.bottom};
  auto bounding_rect = cv::boundingRect(points);

  for (const auto& test_light : lights) {
    if (test_light.center == light_1.center ||
        test_light.center == light_2.center)
      continue;

    if (bounding_rect.contains(test_light.top) ||
        bounding_rect.contains(test_light.bottom) ||
        bounding_rect.contains(test_light.center)) {
      return true;
    }
  }

  return false;
}

ArmorType Detector::isArmor(const Light &light_1, const Light &light_2) {
  // TODO：请在这里完成对装甲板的判断
  // 提示，你可以从detector类中的armorparams中的参数入手

  // Judge armor type
  ArmorType type;
  if (is_armor) {
    type = /* TODO： 请在这里补充判断大小装甲板的判断条件 */ ? ArmorType::LARGE
                                                         : ArmorType::SMALL;
  } else {
    type = ArmorType::INVALID;
  }

  return type;
}

cv::Mat Detector::getAllNumbersImage() {
  if (armors_.empty()) {
    return cv::Mat(cv::Size(20, 28), CV_8UC1);
  } else {
    std::vector<cv::Mat> number_imgs;
    number_imgs.reserve(armors_.size());
    for (auto& armor : armors_) {
      number_imgs.emplace_back(armor.number_img);
    }
    cv::Mat all_num_img;
    cv::vconcat(number_imgs, all_num_img);
    return all_num_img;
  }
}

void Detector::drawResults(cv::Mat &img) {
  // TODO：请绘制下面的注释所指的内容
  // Draw Lights

  // Draw armors

  // Show numbers and confidence
}
