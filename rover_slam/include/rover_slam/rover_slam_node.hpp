#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "sensor_msgs/msg/camera_info.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include <iostream>
#include "opencv4/opencv2/opencv.hpp"
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <g2o/core/optimization_algorithm_gauss_newton.h>

#include "message_filters/subscriber.h"
#include "message_filters/time_synchronizer.h"
#include "message_filters/sync_policies/approximate_time.h"
#include "message_filters/synchronizer.h"

using namespace message_filters;

class RoverSLAM: public rclcpp::Node
{
 public:
  RoverSLAM();
  void initPublishers();
  void initSubscribers();

 private:
  // Topics
  std::string raw_rgb_img_topic_ = "/depth_camera/image_raw";
  std::string raw_depth_img_topic_ = "/depth_camera/depth/image_raw";
  std::string camera_param_topic_ = "/depth_camera/depth/camera_info";

  // Subscriptions
  std::shared_ptr<message_filters::Subscriber<sensor_msgs::msg::Image>> rgb_image_sub_;
  std::shared_ptr<message_filters::Subscriber<sensor_msgs::msg::Image>> depth_image_sub_;
  rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr camera_param_sub_;

  // Sync
  typedef sync_policies::ApproximateTime<sensor_msgs::msg::Image, sensor_msgs::msg::Image> MySyncPolicy;
  std::shared_ptr<message_filters::Synchronizer<MySyncPolicy>> sync_;

  // Publishers
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr state_pub_;

  // Class Variables
  bool intrinsic_param_mat_available_ = false;
  cv::Mat rgb_img_, depth_img_prev_, depth_img_current_;
  std::vector<cv::Mat> img_;
  bool depth_img_available_ = true, rgb_img_available_ = true;
  cv::Ptr<cv::ORB> orb_detector_ = cv::ORB::create();
  cv::Ptr<cv::DescriptorMatcher> matcher_  = cv::DescriptorMatcher::create ("BruteForce-Hamming");
  cv::Mat descriptors_prev_, descriptors_current_;
  std::vector<cv::KeyPoint> keypoints_prev_, keypoints_current_;
  std::vector<cv::DMatch> good_matches_;
  std::vector<cv::Mat> keypoints_matched_prev_, keypoints_matched_current_;
  std::vector<float> depths_matched_prev_, depths_matched_current_;
  float max_match_distance_ = 20; // to be tuned
  bool no_match_flag_ = false;
  // cv::Mat prev_img_; // to visualize the good matches

  std::vector<cv::Point3f> points1_, points2_; // 3D locations of the matched features
  Eigen::Isometry3d transformation_matrix_, rover_states_;
  std::unique_ptr<g2o::OptimizationAlgorithmGaussNewton> solver_;

  // Class Methods
  void decodeImg(const sensor_msgs::msg::Image::ConstSharedPtr& raw_rgb_img_msg, const sensor_msgs::msg::Image::ConstSharedPtr& raw_depth_img_msg);
  void runSLAM();
  bool matchKeypoints();
  void getFeatureCoordinatesIn3D();
  void getDepthInMeters();
  void runBundleAdjustment();
  bool applyORB();
  bool isImageAvailable(const sensor_msgs::msg::Image::ConstSharedPtr& raw_img_msg);
  bool processInitialFrame();
  void setGoodMatches(const std::vector<cv::DMatch> &matches);
  void createSolver();
  void getIntrinsicParameters(const sensor_msgs::msg::CameraInfo::SharedPtr camera_param_sub_);
};
