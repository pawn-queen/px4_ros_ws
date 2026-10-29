#include "px4_powerplant_mission_cpp/offboard.hpp"
namespace powerplant {
void OffboardMissionNode::subscribe_vehicle() {
#define SUB(Type, parameter, qos, body)                                        \
  subscriptions_.push_back(create_subscription<Type>(                          \
      s(parameter), qos, [this](Type::ConstSharedPtr m) body))
  SUB(nav_msgs::msg::Odometry, "localized_odom_topic", reliable_qos(),
      { on_odom(*m); });
  SUB(Status, "vehicle_status_topic", px4_qos(), { on_vehicle(*m); });
  SUB(px4_msgs::msg::VehicleLandDetected, "vehicle_land_detected_topic", px4_qos(),
      { on_land_detected(*m); });
  SUB(std_msgs::msg::String, "localization_status_topic", reliable_qos(),
      { on_localization_status(m->data); });
  SUB(px4_msgs::msg::FailsafeFlags, "failsafe_flags_topic", px4_qos(),
      { failsafe_ = *m; });
  SUB(Ack, "vehicle_command_ack_topic", px4_qos(), { ack_ = *m; });
#undef SUB
}
} // namespace powerplant
