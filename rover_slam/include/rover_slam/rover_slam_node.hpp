#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include <iostream>
#include "opencv4/opencv2/opencv.hpp"
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <g2o/core/optimization_algorithm_gauss_newton.h>

class RoverSLAM: public rclcpp::Node
{
 public:
  RoverSLAM();
  void initPublishers();
  void initSubscribers();

 private:
  // Topics
  std::string raw_rgb_img_topic_ = "/depth_camera/image_raw";

  // Subscriptions
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr rgb_image_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr depth_image_sub_;

  // Publishers
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr state_pub_;

  // Class Variables
  cv:: Mat rgb_img_, depth_img_;
  std::vector<cv::Mat> img_;
  bool depth_img_available_ = true, rgb_img_available_ = true;
  cv::Ptr<cv::ORB> orb_detector_ = cv::ORB::create();
  cv::Ptr<cv::DescriptorMatcher> matcher_  = cv::DescriptorMatcher::create ("BruteForce-Hamming");
  cv::Mat descriptors_prev_, descriptors_current_;
  std::vector<cv::KeyPoint> keypoints_prev_, keypoints_current_;
  std::vector<cv::DMatch> good_matches_;
  float max_match_distance_ = 50; // to be tuned
  bool no_match_flag_ = false;
  // cv::Mat prev_img_; to visualize the good matches

  std::vector<cv::Point3f> points1_, points2_;
  Eigen::Isometry3d transformation_matrix_;
  std::unique_ptr<g2o::OptimizationAlgorithmGaussNewton> solver_;

  // Class Methods
  void decodeRGBImg(const sensor_msgs::msg::Image::SharedPtr raw_rgb_img_msg);
  void decodeDepthImg(const sensor_msgs::msg::Image::SharedPtr raw_depth_img_msg);
  void runSLAM();
  void concatImg();
  void matchKeypoints();
  void runBundleAdjustment();
  bool applyORB();
  bool isImageAvailable(const sensor_msgs::msg::Image::SharedPtr raw_img_msg);
  bool processInitialFrame();
  void setGoodMatches(const std::vector<cv::DMatch> &matches);
  void createSolver();
};
