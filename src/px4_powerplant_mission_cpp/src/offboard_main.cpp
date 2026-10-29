#include "px4_powerplant_mission_cpp/offboard.hpp"
int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  try {
    rclcpp::spin(std::make_shared<powerplant::OffboardMissionNode>());
  } catch (const std::exception &e) {
    std::cerr << e.what() << std::endl;
    rclcpp::shutdown();
    return 1;
  }
  rclcpp::shutdown();
  return 0;
}
