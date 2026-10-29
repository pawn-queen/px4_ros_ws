#pragma once
#include "px4_powerplant_mission_cpp/inspection.hpp"
#include "px4_powerplant_mission_cpp/map_snapshot.hpp"
#include "px4_powerplant_mission_cpp/motion_runtime.hpp"
#include "px4_powerplant_mission_cpp/scan_coverage.hpp"
#include "px4_powerplant_mission_cpp/topology.hpp"
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
#include <px4_msgs/msg/vehicle_land_detected.hpp>
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
  Complete,
  Fault
};
const char *phase_name(Phase p);
enum class TakeoffProgress {
  StartupIntent, ResponseProbe, HandoffAscent, StrictClimb
};
const char *takeoff_progress_name(TakeoffProgress);
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
  PlanningFailure failure = PlanningFailure::None;
  int64_t map_epoch = 0, camera_revision = 0;
  int64_t navigation_domain_revision = 0, task_roi_revision = 0;
  double variance = 0;
  double variance_growth = 0;
  ExplorationConfig configuration;
  Bounds bounds{};
  Vec3 source_start = Vec3::Zero(), home = Vec3::Zero();
  size_t frontier_count = 0, candidate_count = 0;
  size_t roi_unknown_remaining = 0;
  TaskRoiMode roi_mode = TaskRoiMode::Work;
  std::string detail;
  std::shared_ptr<const TopologySnapshot> topology;
  // Motion-feasible geometry establishes the monotonically growing coverage
  // scope even when the remaining flight budget cannot fund another photo.
  std::vector<InspectionView> inspection_classification_views;
  std::vector<InspectionView> inspection_views;
  uint64_t inspection_model_revision = 0;
};
struct PhotoResult {
  JsonValue event;
  bool success = false;
  bool quality_accepted = false;
  PhotoCoverageToken coverage_token;
  int64_t camera_revision = 0;
};
struct RegisteredFrame {
  std::optional<RegisteredTargetObservation> observation;
  sensor_msgs::msg::Image::ConstSharedPtr source;
  double speed_mps = inf, yaw_rate_radps = inf;
  std::string reason;
  int64_t camera_revision = 0, received_steady_ns = 0;
  InspectionCamera depth_camera, rgb_camera;
};
class MotionRuntime;
// ROS callbacks and the mission state have a single writer. Workers receive
// copied inputs only.
class OffboardMissionNode : public ParameterNode {
public:
  OffboardMissionNode();
  ~OffboardMissionNode();

private:
  friend struct MissionTestAccess;
  friend struct P1MissionTestAccess;
  friend struct P2MissionTestAccess;
  friend struct RoiMissionTestAccess;
  friend struct GroundMissionTestAccess;
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
  Vec3 velocity_enu_ = Vec3::Zero();
  Eigen::Matrix3d position_covariance_enu_ =
      Eigen::Matrix3d::Constant(std::numeric_limits<double>::quiet_NaN());
  Eigen::Matrix3d velocity_covariance_enu_ =
      Eigen::Matrix3d::Constant(std::numeric_limits<double>::quiet_NaN());
  double position_sigma_m_ = inf, velocity_sigma_mps_ = inf;
  int64_t odom_stamp_ = 0;
  int64_t odom_sample_ = 0, odom_seen_ = 0, odom_steady_ = 0, last_clock_ = 0;
  double xy_variance_ = inf, z_variance_ = inf, tilt_variance_ = inf,
         yaw_variance_ = inf;
  std::string input_reject_reason_, fault_reason_;
  uint64_t rejected_odom_ = 0;
  int64_t localization_epoch_ = 0, localization_sequence_ = 0,
          localization_stamp_ = 0, localization_steady_ = 0,
          localization_boundary_sample_ = 0;
  bool independent_healthy_ = false;
  std::set<int64_t> retired_localization_epochs_;
  Status vehicle_;
  px4_msgs::msg::VehicleLandDetected land_;
  int64_t vehicle_stamp_ = 0, vehicle_steady_ = 0, land_stamp_ = 0,
          land_steady_ = 0, landed_since_ = 0;
  int64_t land_sample_ = 0;
  bool flight_armed_ = false, flight_offboard_ = false,
       landing_timeout_reported_ = false;
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
  int64_t depth_sample_ = 0, lidar_sample_ = 0, grid_sample_ = 0,
          rgb_sample_ = 0, depth_steady_ = 0, lidar_steady_ = 0,
          grid_steady_ = 0, rgb_steady_ = 0;
  sensor_msgs::msg::CameraInfo::ConstSharedPtr camera_info_;
  std::optional<CameraViewGeometry> fused_camera_;
  int64_t camera_revision_ = 0;
  sensor_msgs::msg::Image::ConstSharedPtr rgb_;
  std::optional<TargetObservation> target_;
  Path waypoints_, navigation_goals_;
  std::vector<int> navigation_ids_;
  size_t waypoint_ = 0;
  std::map<size_t, int> goal_ids_;
  std::set<int> captured_;
  int photos_ = 0;
  int64_t mission_started_ = 0, mission_steady_ = 0, search_started_ = 0,
          landing_started_ = 0, landing_started_steady_ = 0;
  bool landing_sent_ = false;
  std::optional<YawScan> scan_;
  int64_t scan_entered_ = 0, hover_since_ = 0;
  bool rotating_ = false;
  HeadingCoverage scan_coverage_;
  int64_t scan_observation_id_ = 0, scan_observation_epoch_ = 0;
  size_t scan_accepted_observations_ = 0;
  double scan_heading_ = 0;
  int64_t scan_last_accepted_sample_ = 0, scan_heading_started_ = 0;
  double scan_last_accepted_yaw_ = 0;
  std::optional<int64_t> scan_depth_base_, scan_lidar_base_, step_depth_base_,
      step_lidar_base_;
  Keys visited_;
  KeyMap<uint64_t> visited_views_;
  int64_t visited_views_revision_ = 0;
  double explore_yaw_ = 0, explore_score_ = 0;
  int64_t viewpoint_entered_ = 0, viewpoint_depth_base_ = 0;
  int64_t viewpoint_aligned_since_ = 0;
  int64_t viewpoint_revision_ = 0, viewpoint_observation_id_ = 0;
  int64_t latest_depth_observation_id_ = 0,
          latest_depth_observation_sample_ = 0;
  Vec3 latest_depth_observation_position_ = Vec3::Zero();
  double latest_depth_observation_yaw_ = 0;
  int64_t explore_revision_ = -1, return_revision_ = -1, target_revision_ = -1;
  int64_t explore_task_started_ = 0, explore_task_steady_ = 0;
  int64_t explore_plan_stamp_ = 0, return_plan_stamp_ = 0,
          target_plan_stamp_ = 0;
  bool planning_ = false;
  bool explore_global_search_ = false;
  TaskRoiMode roi_mode_ = TaskRoiMode::Work;
  double roi_access_elapsed_s_ = 0;
  int64_t roi_access_last_steady_ = 0;
  size_t roi_unknown_remaining_ = 0;
  int64_t roi_count_revision_ = 0;
  KeyMap<size_t> roi_recent_observations_;
  mutable std::optional<TaskRoi> configured_task_roi_;
  mutable std::array<double, 3> configured_roi_access_policy_{{NAN, NAN, NAN}};
  mutable int64_t task_roi_revision_ = 0;
  mutable std::shared_ptr<const PlanningMap> navigation_map_;
  mutable int64_t navigation_map_observation_ = -1,
                  navigation_map_epoch_ = -1, navigation_domain_revision_ = 0;
  mutable size_t navigation_map_free_count_ = 0,
                 navigation_map_occupied_count_ = 0;
  mutable std::optional<Bounds> navigation_column_bounds_;
  mutable std::optional<Bounds> navigation_observed_bounds_;
  mutable std::vector<double> navigation_column_signature_;
  PlanningFailure explore_failure_ = PlanningFailure::WaitingForMap;
  uint64_t reused_plan_results_ = 0, rejected_plan_results_ = 0;
  Phase planning_phase_ = Phase::Idle;
  std::shared_ptr<const TopologySnapshot> topology_;
  uint64_t selected_region_id_ = 0;
  int64_t topology_built_ = 0;
  std::deque<std::pair<int64_t, uint64_t>> recent_regions_;
  PoseHistory inspection_history_;
  struct TimedKinematics {
    int64_t stamp;
    double speed, yaw_rate;
  };
  std::deque<TimedKinematics> inspection_kinematics_;
  struct DepthFrame {
    int64_t stamp;
    cv::Mat image;
    std::string frame;
  };
  std::deque<DepthFrame> depth_frames_;
  std::deque<sensor_msgs::msg::Image::ConstSharedPtr> source_frames_;
  std::deque<sensor_msgs::msg::CameraInfo::ConstSharedPtr> depth_infos_,
      rgb_infos_;
  std::optional<JsonValue> pending_detection_;
  bool registration_busy_ = false;
  int64_t submitted_observation_ = 0, last_capture_attempt_ = 0;
  InspectionSession inspection_;
  std::optional<RegisteredFrame> registered_frame_;
  std::optional<InspectionView> inspection_view_;
  uint64_t inspection_classified_revision_ = 0;
  std::set<int> attempted_inspection_views_;
  std::string inspection_result_ = "UNCONFIRMED", inspection_wait_;
  int64_t inspection_aligned_since_ = 0;
  int scan_moves_ = 0;
  int64_t scan_move_started_ = 0;
  std::optional<Vec3> scan_original_anchor_;
  Vec3 route_origin_ = Vec3::Zero();
  mutable int64_t motion_stop_since_ = 0;
  uint64_t motion_generation_ = 0, motion_publication_base_ = 0;
  std::shared_ptr<const PlanningMap> motion_map_;
  int64_t motion_map_revision_ = -1, motion_map_epoch_ = -1;
  int64_t motion_map_domain_revision_ = -1;
  Vec3 motion_target_ = Vec3::Zero();
  double motion_target_yaw_ = 0;
  bool motion_permit_ = false;
  std::string motion_input_reason_;
  MotionLimits motion_limits() const;
  MotionLimits strict_motion_limits() const;
  MotionState motion_state() const;
  MotionSafetyDomain motion_domain() const;

  bool motion_brake_requested_ = false;
  bool takeoff_bootstrap_active_ = false;
  TakeoffProgress takeoff_progress_ = TakeoffProgress::StartupIntent;
  double takeoff_transition_budget_s_ = 12.;
  double takeoff_support_clearance_lower_m_ = -inf;
  double takeoff_handoff_rise_m_ = 0.;
  int64_t takeoff_response_since_ = 0, takeoff_response_last_sample_ = 0;
  uint32_t takeoff_response_samples_ = 0;
  bool takeoff_response_confirmed_ = false;
  std::string takeoff_budget_reason_;
  double takeoff_transition_budget_s() const;
  double takeoff_lowest_support_clearance_m() const;
  void update_takeoff_response_evidence();
  std::optional<double> predict_takeoff_handoff_rise() const;
  MotionCertificateKind takeoff_safety_stage_ = MotionCertificateKind::AirborneStrict;
  uint64_t takeoff_authorization_revision_ = 1;
  Vec3 takeoff_ground_reference_ = Vec3::Zero();
  double takeoff_ground_yaw_ = 0;
  int64_t takeoff_armed_ros_ = 0, takeoff_armed_steady_ = 0;
  bool takeoff_strict_requested_ = false;
  std::string takeoff_handoff_reason_;
  std::string ground_support_parameter_reason(
      const std::vector<rclcpp::Parameter> &overrides = {}) const;
  void reset_takeoff_safety();
  void update_takeoff_safety();
  bool handle_motion_failure(const MotionUpdate &);
  bool takeoff_strict_ready() const;
  double takeoff_ground_remaining_s() const;
  int64_t warmup_started_ = 0;
  bool photo_pending_ = false;
  int photo_goal_ = 0;
  bool photo_final_ = false;
  bool battery_connected_ = false;
  double battery_fraction_ = NAN, battery_time_ = NAN;
  int64_t battery_stamp_ = 0;
  std::vector<rclcpp::SubscriptionBase::SharedPtr> subscriptions_;
  rclcpp::Service<Trigger>::SharedPtr start_service_, land_service_;
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr
      roi_parameter_callback_;
  rclcpp::Publisher<px4_msgs::msg::OffboardControlMode>::SharedPtr
      heartbeat_pub_;
  rclcpp::Publisher<px4_msgs::msg::TrajectorySetpoint>::SharedPtr setpoint_pub_;
  rclcpp::Publisher<Command>::SharedPtr command_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr status_pub_, event_pub_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;
  rclcpp::Publisher<geometry_msgs::msg::PointStamped>::SharedPtr target_pub_;
  // Workers are declared last and therefore stop before state/message members
  // are destroyed.
  int64_t map_job_epoch_ = 0, map_job_sequence_ = 0, map_job_stamp_ = 0;
  const sensor_msgs::msg::PointCloud2 *map_job_cloud_ = nullptr;
  double map_job_clearance_ = -1;
  AsyncMapCommitter map_committer_;
  LatestWorker<PlanResult> planner_;
  LatestWorker<PhotoResult> recorder_;
  LatestWorker<RegisteredFrame> registration_;
  std::unique_ptr<MotionRuntime> motion_;
  void update_motion_input(const Vec3 &, double, bool permit,
                           const std::string &reason = "");
  void stop_motion(const std::string &);
  bool motion_stopped() const;
  void refresh_motion_geometry();
  TaskRoi task_roi() const;
  TaskRoi roi_access_region() const;
  void update_roi_access_budget();
  void set_roi_mode(TaskRoiMode);
  void reset_roi_task();
  PlanningMap planning_domain_map() const;
  void invalidate_inspection_calibration(const std::string &);
  void try_registered_observation();
  void consume_registered_observation();
  void cache_depth_frame(const sensor_msgs::msg::Image &, const cv::Mat &);
  void cache_source_frame(sensor_msgs::msg::Image::ConstSharedPtr);
  InspectionConfig inspection_config() const;
  InspectionCamera inspection_camera(const sensor_msgs::msg::CameraInfo &,
                                     bool rgb) const;
  void consume_photo_result();
  void finish_inspection(const std::string &reason);

  void schedule_map_commit();
  void consume_map_commit();
  void reset_p2_epoch();
  void on_odom(const nav_msgs::msg::Odometry &);
  void on_localization_status(const std::string &);
  void on_vehicle(const Status &);
  void on_land_detected(const px4_msgs::msg::VehicleLandDetected &);
  bool observe_clock();
  bool receipt_fresh(int64_t stamp, int64_t steady, double max_age) const;
  bool vehicle_fresh() const;
  std::string motion_health_reason() const;
  std::string takeoff_space_reason() const;
  void fault(const std::string &);
  void on_grid(const nav_msgs::msg::OccupancyGrid &);
  void on_fusion(const std::string &);
  void observe_fused_camera(const JsonValue &);
  void observe_scan_depth(const JsonValue &);
  void observe_viewpoint_depth(const JsonValue &);
  bool scan_first_segment_ready() const;
  bool camera_ready() const;
  bool plan_changes_relevant(const PlanResult &) const;
  void refresh_view_memory();
  void on_detection(const std::string &);
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
  bool exploration_task_budget_exhausted() const;
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
  bool set_local_route(const Path &, double length);
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
