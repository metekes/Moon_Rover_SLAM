#include "rover_slam_node.hpp"
#include "cv_bridge/cv_bridge.h"
#include "icp_graph.hpp"

using std::placeholders::_1;

RoverSLAM::RoverSLAM() : Node("rover_SLAM") {
  initPublishers();
  initSubscribers();
}

void RoverSLAM::initSubscribers() {
  // rmw_qos_profile_t qos_profile = rmw_qos_profile_sensor_data;
  // auto qosSensorData = rclcpp::QoS(rclcpp::QoSInitialization(qos_profile.history, 1), qos_profile);
  rgb_image_sub_ = this->create_subscription<sensor_msgs::msg::Image>(raw_rgb_img_topic_, 5, std::bind(&RoverSLAM::decodeRGBImg, this, std::placeholders::_1));

  depth_image_sub_ = this->create_subscription<sensor_msgs::msg::Image>(raw_rgb_img_topic_, 5, std::bind(&RoverSLAM::decodeDepthImg, this, std::placeholders::_1));
}

void RoverSLAM::initPublishers() {
  // TO BE IMPLEMENTED
}

void RoverSLAM::decodeRGBImg(const sensor_msgs::msg::Image::SharedPtr raw_rgb_img_msg) {
  if (isImageAvailable(raw_rgb_img_msg) == false) {
    rgb_img_available_ = false;
  }

  else {
    rgb_img_available_ = true;
    rgb_img_ = cv_bridge::toCvShare(raw_rgb_img_msg, "bgr8") -> image;
    runSLAM();
  }
}

void RoverSLAM::decodeDepthImg(const sensor_msgs::msg::Image::SharedPtr raw_depth_img_msg) {
  if (isImageAvailable(raw_depth_img_msg) == false) {
    depth_img_available_ = false;
  }
  else {
    depth_img_available_ = true;
    depth_img_ = cv_bridge::toCvShare(raw_depth_img_msg, "32FC1") -> image;
  }
}

void RoverSLAM::runSLAM() {
  concatImg(); // might be deleted
  matchKeypoints();

  // front-end
  runVisualOdometry();

  // back-end
}

void RoverSLAM::concatImg() {
  if (depth_img_available_ && rgb_img_available_) {
    img_.clear();
    img_.push_back(rgb_img_);
    img_.push_back(depth_img_);
  }
}

void RoverSLAM::matchKeypoints() {
  if (applyORB() == false or no_match_flag_ == true) {
    // Do nothing
  }
  else {
    std::vector<cv::DMatch> matches;
    matcher_->match(descriptors_prev_, descriptors_current_, matches);
    setGoodMatches(matches);
  }

  /* TO VISULAIZE MATCHES, SHOULD BE INCLUDED INTO ABOVE ELSE WHEN IT IS ACTIVATED
  cv::Mat img = cv_bridge::toCvShare(raw_img_msg, "bgr8") -> image;
  cv::Mat img_match;
  cv::drawMatches(prev_img_, keypoints_prev_, img, keypoints_current_, good_matches_, img_match);
  prev_img_ = img;
  cv::imshow("good matches", img_match);
  cv::waitKey(1);
  */
}

void RoverSLAM::runVisualOdometry() {
  typedef g2o::BlockSolver<g2o::BlockSolverTraits<6, 3>> Block;
  auto linearSolver = std::make_unique<g2o::LinearSolverEigen<Block::PoseMatrixType>>();
  auto solver_ptr = std::make_unique<Block>(std::move(linearSolver));
  auto solver = std::make_unique<g2o::OptimizationAlgorithmGaussNewton>(std::move(solver_ptr));

  g2o::SparseOptimizer optimizer;
  optimizer.setAlgorithm(solver.get());  // Pass the raw pointer

  // node
  g2o::VertexSE3Expmap* pose = new g2o::VertexSE3Expmap(); // camera pose
  pose->setId(0);
  pose->setEstimate(g2o::SE3Quat(Eigen::Matrix3d::Identity(), Eigen::Vector3d(0,0,0)));
  optimizer.addVertex(pose);

  // edges
  int index = 1;
  std::vector<ICPGraph*> edges;
  for (size_t i = 0; i < points1_.size(); i++) {
      ICPGraph* edge = new ICPGraph(Eigen::Vector3d(points2_[i].x, points2_[i].y, points2_[i].z));
      edge->setId(index);
      edge->setVertex(0, dynamic_cast<g2o::VertexSE3Expmap*>(pose));
      edge->setMeasurement(Eigen::Vector3d(points1_[i].x, points1_[i].y, points1_[i].z));
      edge->setInformation(Eigen::Matrix3d::Identity() * 1e4);  // To be tuned
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
}

bool RoverSLAM::applyORB() {
  
  if (rgb_img_available_ == false) {
    return false;
  }
  
  if (processInitialFrame() == true) {
    return false;
  }

  cv::Mat gray_img;
  cv::cvtColor(rgb_img_, gray_img, CV_BGR2GRAY);
  orb_detector_->detectAndCompute(gray_img, cv::Mat(), keypoints_current_, descriptors_current_);

  // cv::imshow("RGB Image", img);
  // cv::waitKey(1);
  keypoints_prev_   = keypoints_current_;
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

bool RoverSLAM::processInitialFrame() {
  if (keypoints_prev_.empty()) {
    cv::Mat gray_img;
    cv::cvtColor(rgb_img_, gray_img, CV_BGR2GRAY);
    orb_detector_->detectAndCompute(gray_img, cv::Mat(), keypoints_prev_, descriptors_prev_);
    // prev_img_ = img; to visualize the good matches
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