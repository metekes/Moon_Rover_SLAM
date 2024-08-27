#include "rover_slam_node.hpp"
#include "cv_bridge/cv_bridge.h"
#include "icp_graph.hpp"

using std::placeholders::_1;
using std::placeholders::_2;

RoverSLAM::RoverSLAM() : Node("rover_SLAM") {
  rover_states_ = Eigen::Isometry3d::Identity();
  initPublishers();
  initSubscribers();
  createSolver();
}

void RoverSLAM::initSubscribers() {
  rmw_qos_profile_t qos_profile = rmw_qos_profile_sensor_data;
  auto qosSensorData = rclcpp::QoS(rclcpp::QoSInitialization(qos_profile.history, 1), qos_profile);

  depth_image_sub_ = std::make_shared<message_filters::Subscriber<sensor_msgs::msg::Image>>(this, raw_depth_img_topic_);
  rgb_image_sub_ = std::make_shared<message_filters::Subscriber<sensor_msgs::msg::Image>>(this, raw_rgb_img_topic_);

  typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::msg::Image, sensor_msgs::msg::Image> MySyncPolicy;
  sync_ = std::make_shared<message_filters::Synchronizer<MySyncPolicy>>(MySyncPolicy(10),  *rgb_image_sub_, *depth_image_sub_);
  sync_->registerCallback(std::bind(&RoverSLAM::decodeImg, this, _1, _2));

  camera_param_sub_ = this->create_subscription<sensor_msgs::msg::CameraInfo>(camera_param_topic_, qosSensorData, std::bind(&RoverSLAM::getIntrinsicParameters, this, _1));
}

void RoverSLAM::initPublishers() {
  // TO BE IMPLEMENTED
}

void RoverSLAM::decodeImg(const sensor_msgs::msg::Image::ConstSharedPtr& raw_rgb_img_msg, const sensor_msgs::msg::Image::ConstSharedPtr& raw_depth_img_msg) {
  if (isImageAvailable(raw_rgb_img_msg) == false) {
    rgb_img_available_ = false;
    depth_img_available_ = false;
  }

  else {
    rgb_img_available_ = true;
    depth_img_available_ = true;

    rgb_img_ = cv_bridge::toCvShare(raw_rgb_img_msg, "bgr8") -> image;
    depth_img_current_ = cv_bridge::toCvShare(raw_depth_img_msg, "32FC1") -> image;
    /*
    cv::Mat depth_img_normalized;
    cv::normalize(depth_img_current_, depth_img_normalized, 0, 255, cv::NORM_MINMAX);
    depth_img_normalized.convertTo(depth_img_normalized, CV_8UC1); // Convert to 8-bit grayscale
    cv::imshow("depth_img_current", depth_img_normalized);
    cv::waitKey(100);
    */
    runSLAM();
  }
}

void RoverSLAM::runSLAM() {
  if (matchKeypoints()) {
    // front-end
    if (intrinsic_param_mat_available_) {
      getFeatureCoordinatesIn3D();
      runBundleAdjustment();
      std::cout << "here 11" << std::endl;
      rover_states_ = rover_states_ * transformation_matrix_;
      std::cout << "here 2" << std::endl;

      points1_.clear();
      points2_.clear();

      keypoints_matched_prev_.clear();
      keypoints_matched_current_.clear();

      depths_matched_current_.clear();
      depths_matched_prev_.clear();

      keypoints_prev_   = keypoints_current_;
      descriptors_prev_ = descriptors_current_.clone();
    }

    // back-end
  }
}

bool RoverSLAM::matchKeypoints() {
  if (applyORB() == false or no_match_flag_ == true) {
    return false;
  }
  else {
    std::vector<cv::DMatch> matches;
    matcher_->match(descriptors_prev_, descriptors_current_, matches);
    setGoodMatches(matches);
    std::cout << "here 4" << std::endl;

    for (uint i=0; i<good_matches_.size(); i++) {
      int index_prev = good_matches_[i].queryIdx;
      int index_current = good_matches_[i].trainIdx;

      // Previous features in pixel frame
      cv::Mat pixel_prev(3, 1, CV_32S);
      pixel_prev.at<int>(0, 0) = static_cast<int>(keypoints_prev_[index_prev].pt.x);
      pixel_prev.at<int>(1, 0) = static_cast<int>(keypoints_prev_[index_prev].pt.y);
      pixel_prev.at<int>(2, 0) = 1;

      // Current features in pixel frame
      cv::Mat pixel_current(3, 1, CV_32S);
      pixel_current.at<int>(0, 0) = static_cast<int>(keypoints_current_[index_current].pt.x);
      pixel_current.at<int>(1, 0) = static_cast<int>(keypoints_current_[index_current].pt.y);
      pixel_current.at<int>(2, 0) = 1;

      keypoints_matched_prev_.push_back(pixel_prev);
      keypoints_matched_current_.push_back(pixel_current);
    }
    std::cout << "here 6" << std::endl;

    /* TO VISULAIZE MATCHES, SHOULD BE INCLUDED INTO ABOVE ELSE WHEN IT IS ACTIVATED
    cv::Mat img_match;
    cv::drawMatches(prev_img_, keypoints_prev_, rgb_img_, keypoints_current_, good_matches_, img_match);
    prev_img_ = rgb_img_;
    cv::imshow("good matches", img_match);
    cv::waitKey(100);
    */
    return true;
  }
}

void RoverSLAM::getFeatureCoordinatesIn3D() {
  if (intrinsic_param_mat_available_) {
    cv::Mat_<float> intrinsic_param_mat_(3,3);
    intrinsic_param_mat_ << 565.6, 0, 320.5, 0, 565.5, 240.5, 0, 0, 1;
    for (uint i=0; i<keypoints_matched_prev_.size(); i++) {
      // intrincis full 0 dönüyor
      cv::Mat_<float> keypoint_prev_float;
      keypoints_matched_prev_[i].convertTo(keypoint_prev_float, CV_32F);
      cv::Mat result = intrinsic_param_mat_.inv() * keypoint_prev_float;
      points1_.push_back(cv::Point3f(result.at<float>(0, 0), result.at<float>(1, 0), result.at<float>(2, 0)));
    }

    for (uint i=0; i<keypoints_matched_current_.size(); i++) {
      cv::Mat_<float> keypoint_current_float;
      keypoints_matched_current_[i].convertTo(keypoint_current_float, CV_32F);
      cv::Mat result = intrinsic_param_mat_.inv() * keypoint_current_float;
      points2_.push_back(cv::Point3f(result.at<float>(0, 0), result.at<float>(1, 0), result.at<float>(2, 0)));
    }
    getDepthInMeters();
    
    for (uint i=0; i<points1_.size(); i++) {
      points1_[i] = points1_[i] * depths_matched_prev_[i];
    }

    for (uint i=0; i<points2_.size(); i++) {
      points2_[i] = points2_[i] * depths_matched_current_[i];
    }
  }
}

void RoverSLAM::getDepthInMeters() {
  /*
  cv::Mat depth_img_normalized;
  cv::normalize(depth_img_prev_, depth_img_normalized, 0, 255, cv::NORM_MINMAX);
  depth_img_normalized.convertTo(depth_img_normalized, CV_8UC1); // Convert to 8-bit grayscale
  cv::imshow("depth_img_prev", depth_img_normalized);
  cv::waitKey(100);
  */

  for (uint i=0; i<points1_.size(); i++) {
    depths_matched_prev_.push_back(depth_img_prev_.at<float>(keypoints_matched_prev_[i].at<int>(1,0), keypoints_matched_prev_[i].at<int>(0,0)));
  }

  for (uint i=0; i<points2_.size(); i++) {
    depths_matched_current_.push_back(depth_img_current_.at<float>(keypoints_matched_current_[i].at<int>(1,0), keypoints_matched_current_[i].at<int>(0,0)));
  }
  depth_img_prev_ = depth_img_current_.clone();
}

void RoverSLAM::runBundleAdjustment() {
  g2o::SparseOptimizer optimizer;
  optimizer.setAlgorithm(solver_.get());

  // add the node
  g2o::VertexSE3Expmap* pose = new g2o::VertexSE3Expmap(); // camera pose
  pose->setId(0);
  pose->setEstimate(g2o::SE3Quat(Eigen::Matrix3d::Identity(), Eigen::Vector3d(0,0,0)));
  optimizer.addVertex(pose);

  // add the edges
  int index = 1;
  std::vector<ICPGraph*> edges;
  for (size_t i = 0; i < points2_.size(); i++) {
    ICPGraph* edge = new ICPGraph(Eigen::Vector3d(points1_[i].x, points1_[i].y, points1_[i].z));
    edge->setId(index);
    edge->setVertex(0, pose);
    edge->setMeasurement(Eigen::Vector3d(points2_[i].x, points2_[i].y, points2_[i].z));
    edge->setInformation(Eigen::Matrix3d::Identity());  // To be tuned
    optimizer.addEdge(edge);
    index++;
    edges.push_back(edge);
  }

  optimizer.setVerbose(true);
  optimizer.initializeOptimization();
  optimizer.optimize(10);

  transformation_matrix_ = Eigen::Isometry3d(pose->estimate()).matrix();
  // Clean up
  for (auto edge : edges) {
      delete edge;
  }
  delete pose;
  std::cout << "here 1" << std::endl;
}

bool RoverSLAM::applyORB() {
  if (rgb_img_available_ == false || depth_img_available_ == false) {
    return false;
  }
  
  if (processInitialFrame() == true) {
    return false;
  }

  cv::Mat gray_img;
  cv::cvtColor(rgb_img_, gray_img, CV_BGR2GRAY);
  orb_detector_->detectAndCompute(gray_img, cv::Mat(), keypoints_current_, descriptors_current_);
  return true;
}

bool RoverSLAM::isImageAvailable(const sensor_msgs::msg::Image::ConstSharedPtr& raw_img_msg) {
  if (raw_img_msg->data.empty()) {
    return false;
  }
  else {
    return true;
  }
}

bool RoverSLAM::processInitialFrame() {
  std::cout << "here 8" << std::endl;
  if (keypoints_prev_.empty() || depth_img_prev_.empty()) {
    cv::Mat gray_img;
    cv::cvtColor(rgb_img_, gray_img, CV_BGR2GRAY);
    orb_detector_->detectAndCompute(gray_img, cv::Mat(), keypoints_prev_, descriptors_prev_);
    // prev_img_ = rgb_img_; // to visualize the good matches
    depth_img_prev_ = depth_img_current_.clone();
    return true;
  }
  return false;
}

void RoverSLAM::setGoodMatches(const std::vector<cv::DMatch> &matches) {
  // TO DO: first sort then get good matches to reduce the time complexity
  good_matches_.clear();

  for (uint i=0; i<matches.size(); i++) {
    if (matches[i].distance <= max_match_distance_) {
      good_matches_.push_back(matches[i]);
    }
  }

  if (good_matches_.size() == 0) {
    no_match_flag_ = true;
  }
  else {
    no_match_flag_ = false;
    std::sort(good_matches_.begin(), good_matches_.end());
  }
}

void RoverSLAM::createSolver() {
  typedef g2o::BlockSolver<g2o::BlockSolverTraits<6, 3>> Block;
  auto linearSolver = std::make_unique<g2o::LinearSolverEigen<Block::PoseMatrixType>>();
  auto solver_ptr = std::make_unique<Block>(std::move(linearSolver));
  solver_ = std::make_unique<g2o::OptimizationAlgorithmGaussNewton>(std::move(solver_ptr));
}

void RoverSLAM::getIntrinsicParameters(const sensor_msgs::msg::CameraInfo::SharedPtr camera_param_sub_) {
  const std::array<double, 9>& k_array = camera_param_sub_->k;
  // intrinsic_param_mat_ = cv::Mat(3, 3, CV_32F, (void*)k_array.data()); 
  intrinsic_param_mat_available_ = true;
}