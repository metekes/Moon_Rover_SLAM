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
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub_;

  // Publishers
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr state_pub_;

  // Class Variables
  cv::Ptr<cv::ORB> orb_detector_ = cv::ORB::create();
  cv::Ptr<cv::DescriptorMatcher> matcher_  = cv::DescriptorMatcher::create ( "BruteForce-Hamming" );
  cv::Mat descriptors_prev_, descriptors_current_;
  std::vector<cv::KeyPoint> keypoints_prev_, keypoints_current_;
  std::vector<cv::DMatch> good_matches_;
  float max_match_distance_ = 50; // to be tuned

  // Class Methods
  bool applyORB(const sensor_msgs::msg::Image::SharedPtr raw_img_msg);
  void matchKeypoints(const sensor_msgs::msg::Image::SharedPtr raw_img_msg);
  bool isImageAvailable(const sensor_msgs::msg::Image::SharedPtr raw_img_msg);
  bool processInitialFrame(const cv::Mat img);
  void setGoodMatches(const std::vector<cv::DMatch> &matches);
};
