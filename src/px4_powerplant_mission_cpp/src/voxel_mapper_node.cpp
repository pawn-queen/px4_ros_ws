#include "px4_powerplant_mission_cpp/parameters.hpp"
#include "px4_powerplant_mission_cpp/input_validation.hpp"
#include "px4_powerplant_mission_cpp/depth_sampling.hpp"
#include "px4_powerplant_mission_cpp/ros_support.hpp"
#include <deque>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/create_timer.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <variant>
#include <set>
#include <visualization_msgs/msg/marker_array.hpp>

namespace powerplant {
using Odom = nav_msgs::msg::Odometry;
using Image = sensor_msgs::msg::Image;
using Scan = sensor_msgs::msg::LaserScan;
using Cloud = sensor_msgs::msg::PointCloud2;
using Info = sensor_msgs::msg::CameraInfo;
class VoxelMapperNode : public ParameterNode {
public:
  VoxelMapperNode() : ParameterNode("powerplant_voxel_mapper") {
    map_epoch_ = steady_ns();
    declare_voxel_mapper_parameters(*this);
    grid_.resolution = d("voxel_resolution_m");
    grid_.hit = d("hit_log_odds");
    grid_.miss = d("miss_log_odds");
    grid_.occupied_threshold = d("occupied_log_odds_threshold");
    grid_.free_threshold = d("free_log_odds_threshold");
    if (grid_.resolution <= 0 || d("publish_rate_hz") <= 0 ||
        i("depth_stride_px") <= 0 || i("max_publish_points") < 0)
      throw std::invalid_argument(
          "invalid mapping resolution, rate, stride or publication limit");
    if (i("depth_min_stride_px") < 1 ||
        i("depth_min_stride_px") > i("depth_stride_px") ||
        i("depth_max_rays_per_frame") < 1 ||
        !std::isfinite(d("depth_max_ray_spacing_m")) ||
        d("depth_max_ray_spacing_m") <= 0 ||
        !std::isfinite(d("depth_edge_threshold_m")) ||
        d("depth_edge_threshold_m") < 0 ||
        !std::isfinite(d("depth_min_valid_fraction")) ||
        d("depth_min_valid_fraction") < 0 || d("depth_min_valid_fraction") > 1)
      throw std::invalid_argument("invalid adaptive depth sampling limits");
    gate_ = !b("startup_gate_enabled");
    occupied_pub_ =
        create_publisher<Cloud>(s("occupied_voxels_topic"), reliable_qos());
    observed_pub_ =
        create_publisher<Cloud>(s("observed_voxels_topic"), reliable_qos());
    grid_pub_ = create_publisher<nav_msgs::msg::OccupancyGrid>(
        s("occupancy_grid_topic"), reliable_qos());
    marker_pub_ = create_publisher<visualization_msgs::msg::MarkerArray>(
        s("voxel_marker_topic"), reliable_qos());
    status_pub_ = create_publisher<std_msgs::msg::String>(
        s("fusion_status_topic"), reliable_qos());
    subscriptions_.push_back(create_subscription<Odom>(
        s("localized_odom_topic"), reliable_qos(),
        [this](Odom::ConstSharedPtr m) { odom(*m); }));
    subscriptions_.push_back(create_subscription<std_msgs::msg::String>(
        s("localization_status_topic"), reliable_qos(),
        [this](std_msgs::msg::String::ConstSharedPtr m) {
          localization_status(m->data);
        }));
    subscriptions_.push_back(
        create_subscription<Info>(s("depth_camera_info_topic"), sensor_qos(),
                                  [this](Info::ConstSharedPtr m) {
                                    check_clock();
                                    infos_.push_back(m);
                                    if (infos_.size() > 64)
                                      infos_.pop_front();
                                    drain();
                                  }));
    subscriptions_.push_back(create_subscription<Image>(
        s("depth_image_topic"), sensor_qos(),
        [this](Image::ConstSharedPtr m) { queue(m, 0); }));
    subscriptions_.push_back(create_subscription<Cloud>(
        s("pointcloud_topic"), sensor_qos(),
        [this](Cloud::ConstSharedPtr m) { queue(m, 0); }));
    subscriptions_.push_back(create_subscription<Scan>(
        s("laser_scan_topic"), sensor_qos(), [this](Scan::ConstSharedPtr m) {
          if (!std::isfinite(m->time_increment) || m->time_increment < 0)
            return;
          const long double duration =
              std::max(int64_t(0), int64_t(m->ranges.size()) - 1) *
              static_cast<long double>(m->time_increment) * 1e9L;
          if (duration > d("max_sensor_age_s") * 1e9L) {
            ++rejected_sensor_;
            return;
          }
          queue(m, static_cast<int64_t>(std::nearbyint(duration)));
        }));
    // Wall time keeps the health watchdog active while /clock is paused.
    timer_ = create_wall_timer(
        std::chrono::duration<double>(1 / d("publish_rate_hz")),
        [this] { publish_map(); });
  }

private:
  friend struct MapperTestAccess;
  using Measurement = std::variant<Image::ConstSharedPtr, Scan::ConstSharedPtr,
                                   Cloud::ConstSharedPtr>;
  struct Pending {
    Measurement msg;
    int64_t first, last, received, received_steady;
  };
  VoxelGrid grid_;
  PoseHistory history_;
  std::deque<Info::ConstSharedPtr> infos_;
  std::deque<Pending> pending_;
  std::optional<int64_t> depth_selected_, scan_selected_, cloud_selected_, mapping_start_;
  Vec3 position_ = Vec3::Zero();
  Quat orientation_ = Quat::Identity();
  bool have_pose_ = false, gate_ = false;
  int64_t depth_count_ = 0, lidar_count_ = 0, depth_stamp_ = 0,
          lidar_stamp_ = 0;
  int64_t map_epoch_ = 0, status_sequence_ = 0, last_ros_now_ = 0,
          pose_sample_ = 0, pose_received_ = 0, pose_received_steady_ = 0,
          last_odom_seen_ = 0,
          localization_status_sample_ = 0;
  std::optional<int> px4_reset_counter_;
  int64_t source_epoch_ = 0, source_sequence_ = 0,
          source_received_ = 0, source_received_steady_ = 0,
          source_boundary_sample_ = 0;
  std::set<int64_t> retired_source_epochs_;
  bool pose_quality_valid_ = false, external_health_valid_ = false;
  std::string pose_reason_ = "waiting for valid localization", reset_reason_;
  int64_t rejected_odom_ = 0, rejected_sensor_ = 0;
  JsonValue camera_intrinsics_, depth_last_observation_;
  std::deque<JsonValue> depth_observations_;
  std::vector<rclcpp::SubscriptionBase::SharedPtr> subscriptions_;
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<Cloud>::SharedPtr occupied_pub_, observed_pub_;
  rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr grid_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr
      marker_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr status_pub_;
  void clear() {
    grid_.clear();
    depth_count_ = lidar_count_ = depth_stamp_ = lidar_stamp_ = 0;
    camera_intrinsics_ = depth_last_observation_ = JsonValue();
    depth_observations_.clear();
  }
  bool enabled() {
    if (gate_)
      return true;
    if (!b("startup_gate_enabled"))
      return gate_ = true;
    if (!have_pose_ || position_.z() < d("startup_gate_min_height_m") ||
        tilt(orientation_) > d("startup_gate_max_tilt_rad"))
      return false;
    if (b("startup_gate_clear_on_enable"))
      clear();
    gate_ = true;
    mapping_start_ = history_.samples.back().stamp;
    return true;
  }
  void invalidate_pose(const std::string &reason) {
    pose_quality_valid_ = false;
    pose_reason_ = reason;
    have_pose_ = false;
    history_.samples.clear();
    pending_.clear();
  }
  void reset_epoch(const std::string &reason) {
    ++map_epoch_;
    status_sequence_ = 0;
    reset_reason_ = reason;
    clear();
    invalidate_pose(reason);
    infos_.clear();
    depth_selected_.reset();
    scan_selected_.reset();
    cloud_selected_.reset();
    mapping_start_.reset();
    pose_sample_ = pose_received_ = pose_received_steady_ = last_odom_seen_ = 0;
    gate_ = !b("startup_gate_enabled");
  }
  void check_clock() {
    const int64_t current = ns();
    if (last_ros_now_ > 0 && current < last_ros_now_) {
      reset_epoch("ROS clock moved backwards");
      localization_status_sample_ = 0;
      px4_reset_counter_.reset();
      external_health_valid_ = false;
      source_received_ = source_received_steady_ = 0;
    }
    last_ros_now_ = current;
  }
  std::string localization_reason() const {
    if (!pose_quality_valid_)
      return pose_reason_;
    if (!external_health_valid_)
      return "independent localization health is invalid";
    if (!source_epoch_ ||
        !stamp_reason(source_received_, ns(), d("max_localization_age_s"), 0).empty() ||
        (steady_ns() - source_received_steady_) * 1e-9 > d("max_localization_age_s"))
      return "localization health status is stale";
    auto reason = stamp_reason(pose_sample_, ns(), d("max_localization_age_s"),
                               d("max_future_stamp_skew_s"));
    if (!reason.empty())
      return reason;
    reason = stamp_reason(pose_received_, ns(), d("max_localization_age_s"), 0);
    if (!reason.empty())
      return "localization ROS receipt is stale";
    if (pose_received_steady_ <= 0 ||
        (steady_ns() - pose_received_steady_) * 1e-9 >
            d("max_localization_age_s"))
      return "localization steady receipt is stale (clock or stream paused)";
    return {};
  }
  void localization_status(const std::string &text) {
    check_clock();
    try {
      auto p = parse(text);
      for (const auto *name : {"navigation_sample_ns", "status_stamp_ns",
                               "status_sequence", "localization_epoch"})
        if (!p[name].isInt64())
          throw std::invalid_argument("missing localization health epoch/timestamp");
      const int64_t sample = p["navigation_sample_ns"].asInt64();
      const int64_t epoch = p["localization_epoch"].asInt64(),
                    sequence = p["status_sequence"].asInt64();
      if (epoch <= 0 || sequence <= 0 || !p["uwb_input_healthy"].isBool() ||
          !p["has_independent_position"].isBool())
        throw std::invalid_argument("invalid localization health schema");
      if (retired_source_epochs_.count(epoch) ||
          (epoch == source_epoch_ && sequence <= source_sequence_) ||
          !stamp_reason(p["status_stamp_ns"].asInt64(), ns(), d("max_localization_age_s"),
                        d("max_future_stamp_skew_s")).empty())
        return;
      if (source_epoch_ && source_epoch_ != epoch) {
        retired_source_epochs_.insert(source_epoch_);
        reset_epoch("localization coordinate epoch changed");
      }
      if (source_epoch_ != epoch)
        source_boundary_sample_ = sample;
      source_epoch_ = epoch;
      source_sequence_ = sequence;
      source_received_ = ns();
      source_received_steady_ = steady_ns();
      localization_status_sample_ = sample;
      external_health_valid_ = p["uwb_input_healthy"].asBool() &&
                               p["has_independent_position"].asBool();
      if (!external_health_valid_)
        invalidate_pose("independent localization health is invalid");
    } catch (const std::exception &e) {
      external_health_valid_ = false;
      invalidate_pose("invalid localization health status");
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
                          "invalid localization health: %s", e.what());
    }
  }
  void odom(const Odom &msg) {
    check_clock();
    int64_t stamp = stamp_ns(msg.header.stamp);
    // Network delay/duplicates do not mean a ROS clock or estimator reset.
    if (stamp < source_boundary_sample_ || stamp <= last_odom_seen_) {
      ++rejected_odom_;
      if (stamp == last_odom_seen_ &&
          !pose_covariance_reason(msg.pose.covariance, d("max_position_variance_m2"),
                                   d("max_orientation_variance_rad2")).empty())
        invalidate_pose("localization quality downgraded at same sample time");
      return;
    }
    std::string reason;
    if (msg.header.frame_id != s("map_frame") ||
        msg.child_frame_id != s("base_frame"))
      reason = "localization frame mismatch";
    else
      reason = stamp_reason(stamp, ns(), d("max_localization_age_s"),
                            d("max_future_stamp_skew_s"));
    if (reason.empty())
      last_odom_seen_ = stamp;
    const auto &p = msg.pose.pose.position;
    const auto &q = msg.pose.pose.orientation;
    StampedPose pose{stamp, {p.x, p.y, p.z}, {q.w, q.x, q.y, q.z}};
    if (reason.empty() &&
        (!pose.position.allFinite() || !pose.orientation.coeffs().allFinite() ||
         pose.orientation.norm() < 1e-9))
      reason = "invalid localization pose";
    if (reason.empty())
      reason = pose_covariance_reason(msg.pose.covariance,
                                      d("max_position_variance_m2"),
                                      d("max_orientation_variance_rad2"));
    if (!reason.empty()) {
      ++rejected_odom_;
      invalidate_pose(reason);
      return;
    }
    pose.orientation.normalize();
    // Only increasing samples reach PoseHistory: its old-packet reset heuristic
    // can no longer mistake a delayed network packet for a time reset.
    history_.add(pose);
    position_ = pose.position;
    orientation_ = pose.orientation;
    pose_sample_ = stamp;
    pose_received_ = ns();
    pose_received_steady_ = steady_ns();
    have_pose_ = pose_quality_valid_ = true;
    pose_reason_.clear();
    drain();
  }
  template <class T> void queue(T msg, int64_t offset) {
    check_clock();
    if (!localization_reason().empty() || !enabled()) {
      ++rejected_sensor_;
      return;
    }
    const int64_t received = ns(), monotonic = steady_ns();
    int64_t first = stamp_ns(msg->header.stamp);
    if (offset < 0 || offset > int64_t(d("max_sensor_age_s") * 1e9) ||
        first <= 0 || first > std::numeric_limits<int64_t>::max() - offset ||
        !stamp_reason(first, received, d("max_sensor_age_s"),
                      d("max_future_stamp_skew_s")).empty() ||
        !stamp_reason(first + offset, received, d("max_sensor_age_s"),
                      d("max_future_stamp_skew_s")).empty() ||
        (mapping_start_ && first < *mapping_start_)) {
      ++rejected_sensor_;
      return;
    }
    auto *previous = &cloud_selected_;
    double rate = 0;
    if constexpr (std::is_same_v<T, Image::ConstSharedPtr>) {
      previous = &depth_selected_;
      rate = d("depth_fusion_rate_hz");
    } else if constexpr (std::is_same_v<T, Scan::ConstSharedPtr>) {
      previous = &scan_selected_;
      rate = d("lidar_fusion_rate_hz");
    }
    if (*previous &&
        (first <= **previous ||
         (rate > 0 && first - **previous < int64_t(std::nearbyint(1e9 / rate))))) {
      ++rejected_sensor_;
      return;
    }
    if (!history_.samples.empty() && first < history_.samples.front().stamp) {
      ++rejected_sensor_;
      return;
    }
    *previous = first;
    pending_.push_back({msg, first, first + offset, received, monotonic});
    if (pending_.size() > 32) {
      pending_.pop_front();
      ++rejected_sensor_;
    }
    drain();
  }
  Info::ConstSharedPtr matching_info(const Image &image) const {
    Info::ConstSharedPtr before, after;
    int64_t t = stamp_ns(image.header.stamp);
    for (const auto &m : infos_) {
      if (m->width != image.width || m->height != image.height ||
          m->header.frame_id != image.header.frame_id ||
          !std::isfinite(m->k[0]) || !std::isfinite(m->k[4]) || m->k[0] <= 0 ||
          m->k[4] <= 0)
        continue;
      int64_t ts = stamp_ns(m->header.stamp);
      if (ts <= t) {
        if (!before || ts > stamp_ns(before->header.stamp))
          before = m;
      } else if (ts <= t + 50000000 &&
                 (!after || ts < stamp_ns(after->header.stamp)))
        after = m;
    }
    return before ? before : after;
  }
  void drain() {
    if (!localization_reason().empty()) {
      pending_.clear();
      return;
    }
    if (history_.samples.empty())
      return;
    std::deque<Pending> waiting;
    while (!pending_.empty()) {
      auto p = pending_.front();
      pending_.pop_front();
      if (!stamp_reason(p.first, ns(), d("max_sensor_age_s"),
                        d("max_future_stamp_skew_s")).empty() ||
          (steady_ns() - p.received_steady) * 1e-9 > d("max_sensor_age_s")) {
        ++rejected_sensor_;
        continue;
      }
      if (p.first < history_.samples.front().stamp)
        continue;
      if (p.last > history_.samples.back().stamp) {
        if (p.first <= history_.samples.back().stamp + 2000000000)
          waiting.push_back(p);
        continue;
      }
      auto pose = history_.at(p.first);
      if (!pose)
        continue;
      try {
        if (auto image = std::get_if<Image::ConstSharedPtr>(&p.msg)) {
          if (!matching_info(**image) &&
              history_.samples.back().stamp < p.first + 50000000) {
            waiting.push_back(p);
            continue;
          }
          if (insert_depth(**image, *pose) > 0) {
            ++depth_count_;
            depth_stamp_ = p.first;
          }
        } else if (auto scan = std::get_if<Scan::ConstSharedPtr>(&p.msg)) {
          if (insert_scan(**scan, p.first) > 0) {
            ++lidar_count_;
            lidar_stamp_ = p.last;
          }
        } else
          insert_cloud(*std::get<Cloud::ConstSharedPtr>(p.msg), *pose);
      } catch (const std::exception &e) {
        ++rejected_sensor_;
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
                             "discarding sensor input: %s", e.what());
      }
    }
    pending_ = std::move(waiting);
  }
  size_t insert_depth(const Image &msg, const StampedPose &pose) {
    auto depth = depth_image(msg);
    auto info = matching_info(msg);
    if (!info && !infos_.empty())
      return 0;
    double fx = info ? info->k[0] : d("camera_fx"),
           fy = info ? info->k[4] : d("camera_fy");
    double cx = info ? info->k[2] : d("camera_cx"),
           cy = info ? info->k[5] : d("camera_cy");
    if (!std::isfinite(cx) || !std::isfinite(cy))
      throw std::invalid_argument("invalid depth principal point");
    DepthSamplingConfig sampling;
    sampling.adaptive = b("depth_adaptive_sampling_enabled");
    sampling.min_stride = i("depth_min_stride_px");
    sampling.max_stride = i("depth_stride_px");
    sampling.max_rays = i("depth_max_rays_per_frame");
    sampling.ray_spacing = d("depth_max_ray_spacing_m");
    sampling.edge_threshold = d("depth_edge_threshold_m");
    sampling.min_range = d("min_depth_range_m");
    sampling.max_range = d("max_depth_range_m");
    const auto selected = sample_depth(depth, fx, fy, sampling);
    Path points;
    points.reserve(selected.pixels.size());
    std::set<int> yaw_sectors;
    constexpr int coverage_sector_count = 72;
    auto translation = v("camera_translation_flu");
    for (const auto &pixel : selected.pixels) {
      double z = pixel.depth;
      Vec3 body(z, -(pixel.x - cx) * z / fx, -(pixel.y - cy) * z / fy);
      const Vec3 direction = pose.orientation * body;
      points.push_back(pose.position + direction + pose.orientation * translation);
      if (direction.head<2>().norm() > 1e-9) {
        const double angle = wrap_pi(std::atan2(direction.y(), direction.x()));
        yaw_sectors.insert(std::clamp(
            int(std::floor((angle + pi) / (2 * pi) * coverage_sector_count)),
            0, coverage_sector_count - 1));
      }
    }
    const size_t inserted = b("ray_clear_enabled")
               ? grid_.insert_rays(
                     Path(points.size(),
                          pose.position + pose.orientation * translation),
                     points)
               : grid_.insert_points(points);
    if (!inserted)
      return 0;
    // Bounds use pixel edges and the measured principal point. Asymmetric
    // crops/calibrations must not silently become a centred nominal FOV.
    const double hmin = std::atan2(cx - (depth.cols - .5), fx),
                 hmax = std::atan2(cx + .5, fx),
                 vmin = std::atan2(cy - (depth.rows - .5), fy),
                 vmax = std::atan2(cy + .5, fy);
    const bool calibrated = info && !info->header.frame_id.empty();
    camera_intrinsics_ = object();
    camera_intrinsics_["fx"] = fx;
    camera_intrinsics_["fy"] = fy;
    camera_intrinsics_["cx"] = cx;
    camera_intrinsics_["cy"] = cy;
    camera_intrinsics_["width"] = depth.cols;
    camera_intrinsics_["height"] = depth.rows;
    camera_intrinsics_["frame"] = msg.header.frame_id;
    camera_intrinsics_["stamp_ns"] = integer(info ? stamp_ns(info->header.stamp) : 0);
    camera_intrinsics_["camera_info_valid"] = calibrated;
    camera_intrinsics_["horizontal_min_angle_rad"] = hmin;
    camera_intrinsics_["horizontal_max_angle_rad"] = hmax;
    camera_intrinsics_["vertical_min_angle_rad"] = vmin;
    camera_intrinsics_["vertical_max_angle_rad"] = vmax;
    camera_intrinsics_["horizontal_fov_rad"] = hmax - hmin;
    camera_intrinsics_["vertical_fov_rad"] = vmax - vmin;
    auto observation = object();
    observation["sample_id"] = integer(depth_count_ + 1);
    observation["sample_stamp_ns"] = integer(stamp_ns(msg.header.stamp));
    observation["position_enu_m"] = vector_json(pose.position, 6);
    observation["yaw_enu_rad"] = yaw(pose.orientation);
    observation["tilt_rad"] = tilt(pose.orientation);
    observation["valid_pixel_fraction"] = selected.valid_fraction;
    observation["sampled_pixel_count"] = integer(selected.pixels.size());
    observation["candidate_ray_count"] = integer(selected.candidate_rays);
    observation["budget_limited"] = selected.budget_limited;
    observation["camera_frame"] = msg.header.frame_id;
    observation["camera_info_stamp_ns"] = integer(info ? stamp_ns(info->header.stamp) : 0);
    observation["camera_info_valid"] = calibrated;
    observation["width"] = depth.cols;
    observation["height"] = depth.rows;
    observation["horizontal_min_angle_rad"] = hmin;
    observation["horizontal_max_angle_rad"] = hmax;
    observation["vertical_min_angle_rad"] = vmin;
    observation["vertical_max_angle_rad"] = vmax;
    observation["coverage_sector_count"] = coverage_sector_count;
    observation["coverage_qualified"] =
        selected.valid_fraction >= d("depth_min_valid_fraction");
    observation["valid_yaw_sectors"] = array();
    if (observation["coverage_qualified"].asBool())
      for (int sector : yaw_sectors)
        observation["valid_yaw_sectors"].append(sector);
    depth_last_observation_ = observation;
    depth_observations_.push_back(std::move(observation));
    if (depth_observations_.size() > 32)
      depth_observations_.pop_front();
    return inserted;
  }
  size_t insert_scan(const Scan &msg, int64_t stamp) {
    Path points, origins;
    int64_t increment = std::nearbyint(msg.time_increment * 1e9);
    auto translation = v("lidar_translation_flu");
    for (size_t j = 0; j < msg.ranges.size(); ++j) {
      double r = msg.ranges[j];
      if (!std::isfinite(r) || r < msg.range_min || r > msg.range_max)
        continue;
      auto pose = history_.at(stamp + j * increment);
      if (!pose)
        return 0;
      double angle = msg.angle_min + j * double(msg.angle_increment);
      origins.push_back(pose->position + pose->orientation * translation);
      points.push_back(pose->position +
                       pose->orientation *
                           (Vec3(r * std::cos(angle), r * std::sin(angle), 0) +
                            translation));
    }
    return b("ray_clear_enabled") ? grid_.insert_rays(origins, points)
                                  : grid_.insert_points(points);
  }
  void insert_cloud(const Cloud &msg, const StampedPose &pose) {
    auto frame = msg.header.frame_id.empty() ? s("pointcloud_frame")
                                             : msg.header.frame_id;
    if (frame != s("pointcloud_frame") ||
        (frame != s("map_frame") && frame != s("base_frame")))
      return;
    Path points;
    for (const auto &p : read_cloud(msg, false)) {
      Vec3 v(p[0], p[1], p[2]);
      if (!v.allFinite())
        continue;
      points.push_back(frame == s("map_frame")
                           ? v
                           : Vec3(pose.position + pose.orientation * v));
    }
    grid_.insert_points(points);
  }
  void publish_map() {
    check_clock();
    std_msgs::msg::Header h;
    h.stamp = now();
    h.frame_id = s("map_frame");
    const std::string health_reason = localization_reason();
    bool active = health_reason.empty() && enabled();
    auto snapshot = grid_.snapshot(size_t(i("max_publish_points")));
    Cloud observed = cloud_message(h, snapshot.points);
    if (active) {
      auto occupied = grid_.occupied_points(size_t(i("max_publish_points")));
      std::vector<std::array<float, 4>> points;
      for (const auto &p : occupied)
        points.push_back({float(p.x()), float(p.y()), float(p.z()), 1});
      occupied_pub_->publish(cloud_message(h, points, false));
      observed_pub_->publish(observed);
      publish_grid(h);
      visualization_msgs::msg::Marker marker;
      marker.header = h;
      marker.ns = "occupied_voxels";
      marker.id = 0;
      marker.type = marker.CUBE_LIST;
      marker.action = marker.ADD;
      marker.scale.x = marker.scale.y = marker.scale.z =
          d("voxel_resolution_m");
      marker.color.g = .55;
      marker.color.b = .9;
      marker.color.a = .45;
      for (const auto &p : occupied) {
        geometry_msgs::msg::Point q;
        q.x = p.x();
        q.y = p.y();
        q.z = p.z();
        marker.points.push_back(q);
      }
      visualization_msgs::msg::MarkerArray markers;
      markers.markers.push_back(std::move(marker));
      marker_pub_->publish(markers);
    }
    auto status = object();
    status["map_stamp_ns"] = integer(stamp_ns(h.stamp));
    status["map_epoch"] = integer(map_epoch_);
    status["status_sequence"] = integer(++status_sequence_);
    status["epoch_reset_reason"] = reset_reason_;
    status["localization_healthy"] = health_reason.empty();
    status["localization_reason"] = health_reason;
    status["pose_last_sample_stamp_ns"] = integer(pose_sample_);
    status["pose_last_received_stamp_ns"] = integer(pose_received_);
    status["pose_steady_receipt_age_s"] =
        pose_received_steady_ ? number((steady_ns() - pose_received_steady_) * 1e-9) : JsonValue();
    status["rejected_odom_count"] = integer(rejected_odom_);
    status["rejected_sensor_count"] = integer(rejected_sensor_);
    status["observed_cloud_fingerprint"] = cloud_fingerprint(observed);
    status["camera_intrinsics"] = camera_intrinsics_;
    status["depth_last_observation"] = depth_last_observation_;
    status["depth_observations"] = array();
    for (const auto &observation : depth_observations_)
      status["depth_observations"].append(observation);
    status["map_revision"] = integer(grid_.revision);
    status["mapping_enabled"] = active;
    status["voxel_resolution_m"] = grid_.resolution;
    status["observed_bounds_key"] = JsonValue();
    if (snapshot.bounds) {
      auto bounds = array();
      for (const auto &k : *snapshot.bounds)
        for (int v : k)
          bounds.append(v);
      status["observed_bounds_key"] = bounds;
    }
    status["observed_voxel_count"] = integer(snapshot.observed);
    status["observed_voxels_point_count"] = integer(snapshot.points.size());
    status["free_voxel_count"] = integer(snapshot.free);
    status["occupied_voxel_count"] = integer(snapshot.occupied);
    status["uncertain_voxel_count"] = integer(snapshot.uncertain);
    status["observed_voxels_complete"] = snapshot.complete && active;
    status["depth_fused_count"] = integer(depth_count_);
    status["lidar_fused_count"] = integer(lidar_count_);
    status["depth_last_sample_stamp_ns"] = integer(depth_stamp_);
    status["lidar_last_sample_stamp_ns"] = integer(lidar_stamp_);
    publish_json(status_pub_, status);
  }
  void publish_grid(const std_msgs::msg::Header &h) {
    double low = d("grid_z_min_m"), high = d("grid_z_max_m");
    if (b("use_flight_height_grid_band") && have_pose_) {
      double l = std::max(low, position_.z() -
                                   std::max(0.0, d("grid_z_band_below_m")));
      double u = std::min(high, position_.z() +
                                    std::max(0.0, d("grid_z_band_above_m")));
      if (u > l) {
        low = l;
        high = u;
      }
    }
    std::optional<std::array<double, 4>> bounds;
    if (b("use_fixed_grid_bounds"))
      bounds = std::array<double, 4>{d("grid_min_x_m"), d("grid_min_y_m"),
                                     d("grid_max_x_m"), d("grid_max_y_m")};
    auto grid = grid_.project(low, high, bounds);
    if (!grid)
      return;
    nav_msgs::msg::OccupancyGrid m;
    m.header = h;
    m.info.resolution = grid->resolution;
    m.info.width = grid->width;
    m.info.height = grid->height;
    m.info.origin.position.x = grid->x;
    m.info.origin.position.y = grid->y;
    m.info.origin.orientation.w = 1;
    m.data = std::move(grid->data);
    grid_pub_->publish(m);
  }
};
} // namespace powerplant
#ifndef POWERPLANT_MAPPER_TESTING
int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  try {
    rclcpp::spin(std::make_shared<powerplant::VoxelMapperNode>());
  } catch (const std::exception &e) {
    std::cerr << e.what() << std::endl;
    rclcpp::shutdown();
    return 1;
  }
  rclcpp::shutdown();
  return 0;
}

#endif
