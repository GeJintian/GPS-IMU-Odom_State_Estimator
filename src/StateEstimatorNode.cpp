#include "StateEstimator/StateEstimator.hpp"


int main (int argc, char** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<StateEstimator::StateEstimatorNode>());
  rclcpp::shutdown();
  return 0;
}