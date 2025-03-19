#include "optitrack_wrapper.h"

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  auto OptitrackWrapper = std::make_shared<optitrack_wrapper::OptitrackWrapper>();
  OptitrackWrapper->SetupNatNet();
  rclcpp::spin(OptitrackWrapper);
  rclcpp::shutdown();
}
