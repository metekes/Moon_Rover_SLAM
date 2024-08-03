#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "sensor_msgs/msg/joint_state.hpp"

#include <iostream>
#include "opencv4/opencv2/opencv.hpp"

class RoverSLAM: public rclcpp::Node
{
 public:
  RoverSLAM();
  void initPublishers();
  void initSubscribers();

 private:
  // Topics
  std::string raw_img_topic_ = "/depth_camera/image_raw";

  // Subscriptions
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub_;  // Changed to SharedPtr

  // Publishers
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr state_pub_;  // Changed to SharedPtr

  // Class Variables
  int img_features_;

  // Class Methods
  void getFeatures(const sensor_msgs::msg::Image::SharedPtr raw_img_msg);  // Changed return type to void
};
