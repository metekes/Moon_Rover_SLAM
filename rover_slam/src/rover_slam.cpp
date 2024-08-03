#include <memory>
#include "rclcpp/rclcpp.hpp"
#include "rover_slam_node.hpp"

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<RoverSLAM>());
  rclcpp::shutdown();
  return 0;
}