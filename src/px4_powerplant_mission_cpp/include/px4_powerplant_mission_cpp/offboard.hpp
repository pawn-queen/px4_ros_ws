#pragma once
#include "px4_powerplant_mission_cpp/map_snapshot.hpp"
#include "px4_powerplant_mission_cpp/worker.hpp"
#include <geometry_msgs/msg/point_stamped.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>
#include <px4_msgs/msg/battery_status.hpp>
#include <px4_msgs/msg/failsafe_flags.hpp>
#include <px4_msgs/msg/offboard_control_mode.hpp>
#include <px4_msgs/msg/trajectory_setpoint.hpp>
#include <px4_msgs/msg/vehicle_command.hpp>
#include <px4_msgs/msg/vehicle_command_ack.hpp>
#include <px4_msgs/msg/vehicle_status.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <set>
#include <std_srvs/srv/trigger.hpp>

namespace powerplant {
enum class Phase {
  Idle,
  Start,
  Takeoff,
  Scan,
  Explore,
  Target,
  Return,
  Landing,
  Complete
};
const char *phase_name(Phase p);
struct TargetObservation {
  std::string class_name;
  double confidence = 0;
  Vec3 position = Vec3::Zero();
  int64_t stamp = 0;
};
struct PlanResult {
  Phase phase = Phase::Idle;
  int64_t revision = -1;
  std::optional<ExplorationPlan> frontier;
  Path path;
  std::map<size_t, int> goal_ids;
  std::string error;
};
struct PhotoResult {
  JsonValue event;
  bool success = false;
};
// ROS callbacks and the mission state have a single writer. Workers receive
// copied inputs only.
class OffboardMissionNode : public ParameterNode {
public:
  OffboardMissionNode();

private:
  friend struct MissionTestAccess;
  void subscribe_vehicle();
  void subscribe_map();
  void subscribe_camera();
  using Status = px4_msgs::msg::VehicleStatus;
  using Command = px4_msgs::msg::VehicleCommand;
  using Ack = px4_msgs::msg::VehicleCommandAck;
  using Trigger = std_srvs::srv::Trigger;
  Phase phase_ = Phase::Idle;
  bool active_ = false;
  std::optional<Vec3> position_, home_, return_goal_, explore_goal_,
      scan_anchor_;
  Quat orientation_ = Quat::Identity();
  double yaw_ = 0, speed_ = inf, yaw_rate_ = inf;
  int64_t odom_stamp_ = 0;
  double xy_variance_ = inf, z_variance_ = inf, tilt_variance_ = inf;
  Status vehicle_;
  px4_msgs::msg::FailsafeFlags failsafe_;
  std::optional<Ack> ack_;
  SlewState slew_;
  int warmup_ = 0;
  int64_t offboard_request_ = 0, arm_request_ = 0;
  std::string command_wait_, return_reason_, explore_wait_, scan_wait_,
      safety_reason_;
  bool safety_stop_ = false, avoidance_gate_ = false;
  MapSnapshotReceiver maps_;
  std::optional<Grid2D> grid_;
  int64_t grid_stamp_ = 0;
  size_t grid_known_ = 0;
  cv::Mat depth_;
  int64_t depth_stamp_ = 0, lidar_stamp_ = 0;
  sensor_msgs::msg::CameraInfo::ConstSharedPtr camera_info_;
  sensor_msgs::msg::Image::ConstSharedPtr rgb_;
  std::optional<TargetObservation> target_;
  Path waypoints_, navigation_goals_;
  std::vector<int> navigation_ids_;
  size_t waypoint_ = 0;
  std::map<size_t, int> goal_ids_;
  std::set<int> captured_;
  int photos_ = 0;
  int64_t mission_started_ = 0, search_started_ = 0, landing_started_ = 0;
  bool landing_sent_ = false;
  std::optional<YawScan> scan_;
  int64_t scan_entered_ = 0, hover_since_ = 0;
  bool rotating_ = false;
  std::optional<int64_t> scan_depth_base_, scan_lidar_base_, step_depth_base_,
      step_lidar_base_;
  Keys visited_;
  double explore_yaw_ = 0, explore_score_ = 0;
  int64_t viewpoint_entered_ = 0, viewpoint_depth_base_ = 0;
  int64_t explore_revision_ = -1, return_revision_ = -1, target_revision_ = -1;
  int64_t explore_plan_stamp_ = 0, return_plan_stamp_ = 0,
          target_plan_stamp_ = 0;
  bool planning_ = false;
  Phase planning_phase_ = Phase::Idle;
  bool photo_pending_ = false;
  int photo_goal_ = 0;
  bool photo_final_ = false;
  bool battery_connected_ = false;
  double battery_fraction_ = NAN, battery_time_ = NAN;
  int64_t battery_stamp_ = 0;
  std::vector<rclcpp::SubscriptionBase::SharedPtr> subscriptions_;
  rclcpp::Service<Trigger>::SharedPtr start_service_, land_service_;
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<px4_msgs::msg::OffboardControlMode>::SharedPtr
      heartbeat_pub_;
  rclcpp::Publisher<px4_msgs::msg::TrajectorySetpoint>::SharedPtr setpoint_pub_;
  rclcpp::Publisher<Command>::SharedPtr command_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr status_pub_, event_pub_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;
  rclcpp::Publisher<geometry_msgs::msg::PointStamped>::SharedPtr target_pub_;
  // Workers are declared last and therefore stop before state/message members
  // are destroyed.
  LatestWorker<PlanResult> planner_;
  LatestWorker<PhotoResult> recorder_;
  void on_odom(const nav_msgs::msg::Odometry &);
  void on_grid(const nav_msgs::msg::OccupancyGrid &);
  void on_fusion(const std::string &);
  void on_detection(const std::string &);
  std::optional<Vec3> project_bbox(const JsonValue &, int width,
                                   int height) const;
  void start(Trigger::Response &);
  void tick();
  void initialize();
  void reset_scan();
  void enter_scan(const Vec3 &);
  void update_scan();
  std::pair<bool, std::string> scan_pose_ready(bool hover) const;
  std::pair<bool, std::string> scan_data_ready() const;
  bool localization_ready() const;
  bool tilt_ready() const;
  bool map_fresh() const;
  double remaining_flight() const;
  bool battery_low() const;
  bool reached(const Vec3 &, double tolerance = -1) const;
  bool target_recent() const;
  std::optional<Vec3> home_air() const;
  double face(const Vec3 &) const;
  ExplorationConfig config() const;
  const PlanningMap &planning_map() const;
  void cancel_jobs();
  void enter_explore();
  void enter_target();
  void enter_return(const std::string &);
  void enter_landing(const std::string &);
  void update_landing();
  void prepare_explore();
  void prepare_return();
  void prepare_target();
  void submit_plan(Phase phase, const Path &goals = {},
                   const std::vector<int> &ids = {});
  bool remaining_path_safe() const;
  void set_local_route(const Path &, double length);
  Path inspection_goals(const Vec3 &) const;
  std::pair<Path, std::vector<int>> remaining_goals() const;
  std::pair<Vec3, double> select_target();
  void update_phase(const Vec3 &, double);
  void update_target(const Vec3 &, double);
  bool capture_photo(size_t, int);
  std::pair<bool, std::string> safety_required(const Vec3 &);
  bool avoidance_enabled();
  void safety(bool, const std::string &);
  void reset_setpoint();
  void heartbeat();
  void setpoint(const Vec3 &, double);
  void engage();
  std::string preflight_reason() const;
  void command(uint16_t, double p1 = 0, double p2 = 0);
  void publish_status(std::optional<Vec3> target = {},
                      std::optional<Vec3> setpoint = {});
  void publish_path();
  void event(const std::string &, JsonValue fields = object());
  static std::string command_name(int);
  static std::string result_name(int);
};
} // namespace powerplant
