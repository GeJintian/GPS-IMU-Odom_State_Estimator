#include "StateEstimator/StateEstimator.hpp"


int main (int argc, char** argv)
{
  ros::init(argc, argv, "StateEstimator");
  //ros::NodeHandle n;
  autorally_core::StateEstimator wpt;
  ros::spin();
}