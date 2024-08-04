#include <memory>
#include <opencv2/opencv.hpp>
#include <string>
#include <vector>

#include "armor.hpp"
#include "detector.hpp"
#include "number_classifier.hpp"

std::unique_ptr<Detector> initDetector();
std::vector<Armor> detectArmors(const cv::Mat& img);

cv::VideoCapture cap;
std::unique_ptr<Detector> detector_;

bool debug_;