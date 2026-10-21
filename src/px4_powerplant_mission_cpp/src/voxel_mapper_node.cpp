#include "px4_powerplant_mission_cpp/parameters.hpp"
#include "px4_powerplant_mission_cpp/ros_support.hpp"
#include <deque>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/create_timer.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <variant>
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
    subscriptions_.push_back(
        create_subscription<Info>(s("depth_camera_info_topic"), sensor_qos(),
                                  [this](Info::ConstSharedPtr m) {
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
          int64_t offset = std::max(int64_t(0), int64_t(m->ranges.size()) - 1) *
                           int64_t(std::nearbyint(m->time_increment * 1e9));
          queue(m, offset);
        }));
    timer_ = rclcpp::create_timer(
        this, get_clock(),
        rclcpp::Duration::from_seconds(1 / d("publish_rate_hz")),
        [this] { publish_map(); });
  }

private:
  using Measurement = std::variant<Image::ConstSharedPtr, Scan::ConstSharedPtr,
                                   Cloud::ConstSharedPtr>;
  struct Pending {
    Measurement msg;
    int64_t first, last;
  };
  VoxelGrid grid_;
  PoseHistory history_;
  std::deque<Info::ConstSharedPtr> infos_;
  std::deque<Pending> pending_;
  std::optional<int64_t> depth_selected_, scan_selected_, mapping_start_;
  Vec3 position_ = Vec3::Zero();
  Quat orientation_ = Quat::Identity();
  bool have_pose_ = false, gate_ = false;
  int64_t depth_count_ = 0, lidar_count_ = 0, depth_stamp_ = 0,
          lidar_stamp_ = 0;
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
  void odom(const Odom &msg) {
    int64_t stamp = stamp_ns(msg.header.stamp);
    if (stamp <= 0 ||
        (!msg.header.frame_id.empty() &&
         msg.header.frame_id != s("map_frame")) ||
        (!msg.child_frame_id.empty() && msg.child_frame_id != s("base_frame")))
      return;
    const auto &p = msg.pose.pose.position;
    const auto &q = msg.pose.pose.orientation;
    StampedPose pose{stamp, {p.x, p.y, p.z}, {q.w, q.x, q.y, q.z}};
    if (!pose.position.allFinite() || !pose.orientation.coeffs().allFinite() ||
        pose.orientation.norm() < 1e-9)
      return;
    for (int idx : {21, 28})
      if (!std::isfinite(msg.pose.covariance[idx]) ||
          msg.pose.covariance[idx] < 0 ||
          msg.pose.covariance[idx] > d("max_orientation_variance_rad2"))
        return;
    pose.orientation.normalize();
    if (history_.add(pose)) {
      pending_.clear();
      infos_.clear();
      depth_selected_.reset();
      scan_selected_.reset();
      clear();
      gate_ = !b("startup_gate_enabled");
      mapping_start_.reset();
    }
    position_ = history_.samples.back().position;
    orientation_ = history_.samples.back().orientation;
    have_pose_ = true;
    drain();
  }
  template <class T> void queue(T msg, int64_t offset) {
    if (!enabled())
      return;
    int64_t first = stamp_ns(msg->header.stamp);
    if (first <= 0 || (mapping_start_ && first < *mapping_start_))
      return;
    if (!history_.samples.empty() &&
        (first < history_.samples.front().stamp ||
         first + offset > history_.samples.back().stamp + 2000000000))
      return;
    if constexpr (std::is_same_v<T, Image::ConstSharedPtr> ||
                  std::is_same_v<T, Scan::ConstSharedPtr>) {
      constexpr bool depth = std::is_same_v<T, Image::ConstSharedPtr>;
      auto &previous = depth ? depth_selected_ : scan_selected_;
      double rate = d(depth ? "depth_fusion_rate_hz" : "lidar_fusion_rate_hz");
      if (previous && rate > 0 &&
          first - *previous < int64_t(std::nearbyint(1e9 / rate)))
        return;
      previous = first;
    }
    pending_.push_back({msg, first, first + offset});
    if (pending_.size() > 32)
      pending_.pop_front();
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
    if (history_.samples.empty())
      return;
    std::deque<Pending> waiting;
    while (!pending_.empty()) {
      auto p = pending_.front();
      pending_.pop_front();
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
    Path points;
    auto translation = v("camera_translation_flu");
    for (int y = 0; y < depth.rows; y += i("depth_stride_px"))
      for (int x = 0; x < depth.cols; x += i("depth_stride_px")) {
        double z = depth.at<float>(y, x);
        if (!std::isfinite(z) || z < d("min_depth_range_m") ||
            z > d("max_depth_range_m"))
          continue;
        Vec3 body(z, -(x - cx) * z / fx, -(y - cy) * z / fy);
        points.push_back(pose.position +
                         pose.orientation * (body + translation));
      }
    return b("ray_clear_enabled")
               ? grid_.insert_rays(
                     Path(points.size(),
                          pose.position + pose.orientation * translation),
                     points)
               : grid_.insert_points(points);
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
    std_msgs::msg::Header h;
    h.stamp = now();
    h.frame_id = s("map_frame");
    bool active = enabled();
    auto snapshot = grid_.snapshot(size_t(i("max_publish_points")));
    if (active) {
      auto occupied = grid_.occupied_points(size_t(i("max_publish_points")));
      std::vector<std::array<float, 4>> points;
      for (const auto &p : occupied)
        points.push_back({float(p.x()), float(p.y()), float(p.z()), 1});
      occupied_pub_->publish(cloud_message(h, points, false));
      observed_pub_->publish(cloud_message(h, snapshot.points));
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
    status["map_revision"] = integer(grid_.revision);
    status["mapping_enabled"] = gate_;
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
    status["observed_voxels_complete"] = snapshot.complete && gate_;
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
