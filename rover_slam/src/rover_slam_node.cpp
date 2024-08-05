#include "rover_slam_node.hpp"
#include "cv_bridge/cv_bridge.h"

using std::placeholders::_1;

RoverSLAM::RoverSLAM() : Node("rover_SLAM") {
  initPublishers();
  initSubscribers();
}

void RoverSLAM::initSubscribers() {
  // rmw_qos_profile_t qos_profile = rmw_qos_profile_sensor_data;
  // auto qosSensorData = rclcpp::QoS(rclcpp::QoSInitialization(qos_profile.history, 1), qos_profile);
  image_sub_ = this->create_subscription<sensor_msgs::msg::Image>(raw_img_topic_, 5, std::bind(&RoverSLAM::matchKeypoints, this, std::placeholders::_1));
}

void RoverSLAM::initPublishers() {
  // TO BE IMPLEMENTED
}

void RoverSLAM::matchKeypoints(const sensor_msgs::msg::Image::SharedPtr raw_img_msg) {
  if (applyORB(raw_img_msg) == false) {
    // Do nothing
  }
  else {
    std::vector<cv::DMatch> matches;
    matcher_->match(descriptors_prev_, descriptors_current_, matches);
    setGoodMatches(matches);
  }
}

bool RoverSLAM::applyORB(const sensor_msgs::msg::Image::SharedPtr raw_img_msg) {
  
  if (isImageAvailable(raw_img_msg) == false) {
    return false;
  }

  cv::Mat img = cv_bridge::toCvShare(raw_img_msg, "bgr8") -> image;
  
  if (processInitialFrame(img) == true) {
    return false;
  }

  cv::Mat gray_img;
  cv::cvtColor(img, gray_img, CV_BGR2GRAY);
  orb_detector_->detectAndCompute(img, cv::Mat(), keypoints_current_, descriptors_current_);

  // cv::imshow("RGB Image", img);
  // cv::waitKey(1);
  keypoints_prev_ = keypoints_current_;
  descriptors_prev_ = descriptors_current_;
  return true;
}

bool RoverSLAM::isImageAvailable(const sensor_msgs::msg::Image::SharedPtr raw_img_msg) {
  if (raw_img_msg->data.empty()) {
    return false;
  }
  else {
    return true;
  }
}

bool RoverSLAM::processInitialFrame(const cv::Mat img) {
  if (keypoints_prev_.empty()) {
    cv::Mat gray_img;
    cv::cvtColor(img, gray_img, CV_BGR2GRAY);
    orb_detector_->detectAndCompute(img, cv::Mat(), keypoints_prev_, descriptors_prev_);
    return true;
  }
  return false;
}

void RoverSLAM::setGoodMatches(const std::vector<cv::DMatch> &matches) {
  // TO DO: first sort then get good matches to reduce the time complexity
  good_matches_.clear();
  for (uint i=0; i<=matches.size(); i++) {
    good_matches_.push_back(matches[i]);
  }
  std::sort(good_matches_.begin(), good_matches_.end());
}