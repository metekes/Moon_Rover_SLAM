#include "rover_slam_node.hpp"
#include "cv_bridge/cv_bridge.h"

using std::placeholders::_1;

RoverSLAM::RoverSLAM() : Node("rover_SLAM") {
  initPublishers();
  initSubscribers();
}

void RoverSLAM::initPublishers() {
  // rmw_qos_profile_t qos_profile = rmw_qos_profile_sensor_data;
  // auto qosSensorData = rclcpp::QoS(rclcpp::QoSInitialization(qos_profile.history, 1), qos_profile);
  image_sub_ = this->create_subscription<sensor_msgs::msg::Image>(raw_img_topic_, 5, std::bind(&RoverSLAM::getFeatures, this, std::placeholders::_1));
}

void RoverSLAM::initSubscribers() {
  
}

void RoverSLAM::getFeatures(const sensor_msgs::msg::Image::SharedPtr raw_img_msg) {
  /*
  if (image_msg->image.data.empty()) {
    return false;
  }
  */

  cv::Mat img = cv_bridge::toCvShare(raw_img_msg, "bgr8") -> image;
  cv::imshow("RGB Image", img);
  cv::waitKey(1);
}