#include "px4_powerplant_mission_cpp/map_snapshot.hpp"
#include "px4_powerplant_mission_cpp/input_validation.hpp"
#include "px4_powerplant_mission_cpp/planning_index.hpp"
namespace powerplant {
bool MapSnapshotReceiver::update_status(const std::string &text, int64_t now,
                                        int64_t monotonic_now) {
  if (!monotonic_now)
    monotonic_now = steady_ns();
  JsonValue p;
  try {
    p = parse(text);
    for (const auto *name : {"map_epoch", "status_sequence", "map_stamp_ns",
                            "depth_fused_count", "lidar_fused_count",
                            "depth_last_sample_stamp_ns",
                            "lidar_last_sample_stamp_ns",
                            "pose_last_sample_stamp_ns",
                            "pose_last_received_stamp_ns"})
      if (!p[name].isInt64())
        throw std::invalid_argument(std::string("missing map health field: ") + name);
    if (!p["observed_cloud_fingerprint"].isString() ||
        p["observed_cloud_fingerprint"].asString().empty() ||
        !p["localization_healthy"].isBool() ||
        p["map_epoch"].asInt64() <= 0 ||
        p["status_sequence"].asInt64() <= 0 ||
        p["map_stamp_ns"].asInt64() <= 0 ||
        p["depth_fused_count"].asInt64() < 0 ||
        p["lidar_fused_count"].asInt64() < 0)
      throw std::invalid_argument("invalid map health schema");
  } catch (...) {
    complete = false;
    localization_healthy = false;
    throw;
  }
  const int64_t epoch = p["map_epoch"].asInt64();
  const int64_t sequence = p["status_sequence"].asInt64();
  if (retired_epochs_.count(epoch) ||
      (status_received && epoch == map_epoch && sequence <= status_sequence))
    return false; // A late status must neither refresh nor roll back health.
  const bool restarted = status_received && epoch != map_epoch;
  if (restarted) {
    retired_epochs_.insert(map_epoch);
    complete = false;
    revision = -1;
    stamp = received = received_steady = 0;
    depth_count = lidar_count = depth_sample = lidar_sample = 0;
    depth_received = lidar_received = 0;
    depth_received_steady = lidar_received_steady = 0;
    map.free.clear();
    map.occupied.clear();
    map.index.reset();
    ++observation_revision;
    deltas_.clear();
    delta_cells_ = 0;
    delta_floor_ = observation_revision;
    // A cloud has no epoch field. Never pair a pre-reset cached cloud with a
    // post-reset status, even if ROS time happens to repeat after a reset.
    cloud.reset();
  }
  const int64_t dc = p["depth_fused_count"].asInt64();
  const int64_t lc = p["lidar_fused_count"].asInt64();
  const int64_t ds = p["depth_last_sample_stamp_ns"].asInt64();
  const int64_t ls = p["lidar_last_sample_stamp_ns"].asInt64();
  if ((status_received && !restarted && (dc < depth_count || lc < lidar_count)) ||
      (dc > 0 && ds <= 0) || (lc > 0 && ls <= 0) ||
      (dc > depth_count && ds <= depth_sample) ||
      (lc > lidar_count && ls <= lidar_sample)) {
    complete = false;
    localization_healthy = false;
    throw std::invalid_argument("fusion counters or samples regressed within epoch");
  }
  if (dc > depth_count) {
    depth_received = now;
    depth_received_steady = monotonic_now;
  }
  if (lc > lidar_count) {
    lidar_received = now;
    lidar_received_steady = monotonic_now;
  }
  map_epoch = epoch;
  status_sequence = sequence;
  depth_count = dc;
  lidar_count = lc;
  depth_sample = ds;
  lidar_sample = ls;
  pose_sample = p["pose_last_sample_stamp_ns"].asInt64();
  pose_received = p["pose_last_received_stamp_ns"].asInt64();
  localization_healthy = p["localization_healthy"].asBool();
  status_received = true;
  status = p;
  if (!localization_healthy ||
      !p.get("observed_voxels_complete", false).asBool() ||
      p["map_stamp_ns"].asInt64() != stamp)
    complete = false;
  return restarted;
}
bool MapSnapshotReceiver::commit(const std::string &frame, int64_t now,
                                 int64_t monotonic_now) {
  if (!monotonic_now)
    monotonic_now = steady_ns();
  if (!cloud || !status)
    return false;
  const auto &p = *status;
  int64_t ts = stamp_ns(cloud->header.stamp);
  if (!p["map_stamp_ns"].isInt64() || ts != p["map_stamp_ns"].asInt64() ||
      cloud->header.frame_id != frame || p["map_epoch"].asInt64() != map_epoch ||
      cloud_fingerprint(*cloud) != p["observed_cloud_fingerprint"].asString())
    return false;
  if (!p["observed_voxels_complete"].asBool() ||
      !p["mapping_enabled"].asBool() || !localization_healthy) {
    complete = false;
    return false;
  }
  for (const auto *name :
       {"map_revision", "observed_voxel_count", "observed_voxels_point_count",
        "free_voxel_count", "occupied_voxel_count", "uncertain_voxel_count"})
    if (!p[name].isInt64())
      return false;
  if (!p["voxel_resolution_m"].isNumeric())
    return false;
  double resolution = p["voxel_resolution_m"].asDouble();
  auto rev = p["map_revision"].asInt64(),
       expected = p["observed_voxel_count"].asInt64();
  if (!std::isfinite(resolution) || resolution <= 0 || rev < revision ||
      expected < 0 || expected != p["observed_voxels_point_count"].asInt64() ||
      (stamp > 0 && ts < stamp))
    return false;
  auto rows = read_cloud(*cloud, true);
  if (int64_t(rows.size()) != expected)
    return false;
  Keys observed, free, occupied;
  for (const auto &row : rows) {
    Vec3 v(row[0], row[1], row[2]);
    float state = row[3];
    if (!v.allFinite() || (state != -1 && state != 0 && state != 1))
      return false;
    auto k = key(v, resolution);
    if (!observed.insert(k).second)
      return false;
    if (state == 0)
      free.insert(k);
    else if (state == 1)
      occupied.insert(k);
  }
  if (int64_t(free.size()) != p["free_voxel_count"].asInt64() ||
      int64_t(occupied.size()) != p["occupied_voxel_count"].asInt64() ||
      expected - int64_t(free.size()) - int64_t(occupied.size()) !=
          p["uncertain_voxel_count"].asInt64())
    return false;
  // Wire snapshots are still full maps, so classification diffing is O(N).
  // Frontier/clearance work downstream is limited to the dependency region of
  // these changed classifications; unchanged probability updates do no work.
  Keys changed;
  for (const auto &k : free)
    if (!map.free.count(k))
      changed.insert(k);
  for (const auto &k : map.free)
    if (!free.count(k))
      changed.insert(k);
  for (const auto &k : occupied)
    if (!map.occupied.count(k))
      changed.insert(k);
  for (const auto &k : map.occupied)
    if (!occupied.count(k))
      changed.insert(k);
  auto index = PlanningIndex::update(map.index, free, occupied, changed,
                                     resolution, index_clearance_radius_m);
  const bool resolution_changed = resolution != map.resolution;
  if (!changed.empty() || resolution_changed)
    ++observation_revision;
  if (resolution_changed) {
    // The same integer voxel key now denotes different physical space. No key
    // delta union can validate a plan created at the former resolution.
    deltas_.clear();
    delta_cells_ = 0;
    delta_floor_ = observation_revision;
  } else if (!changed.empty()) {
    delta_cells_ += changed.size();
    deltas_.push_back({observation_revision, std::move(changed)});
    // Bound memory as well as number of publications. Losing history forces
    // conservative replanning, never an assumption that an ROI stayed unchanged.
    while (deltas_.size() > 64 || delta_cells_ > 200000) {
      delta_floor_ = deltas_.front().revision;
      delta_cells_ -= deltas_.front().changed.size();
      deltas_.pop_front();
    }
  }
  map.free = std::move(free);
  map.occupied = std::move(occupied);
  map.resolution = resolution;
  map.index = std::move(index);
  revision = rev;
  stamp = ts;
  received = now;
  received_steady = monotonic_now;
  complete = true;
  cloud.reset();
  status.reset();
  return true;
}
bool MapSnapshotReceiver::fresh(int64_t now, double max_age,
                                int64_t monotonic_now) const {
  if (!monotonic_now)
    monotonic_now = steady_ns();
  auto recent = [&](int64_t t) {
    return t > 0 && now >= t && (now - t) * 1e-9 <= max_age;
  };
  auto monotonic_recent = [&](int64_t t) {
    return t > 0 && monotonic_now >= t &&
           (monotonic_now - t) * 1e-9 <= max_age;
  };
  return complete && localization_healthy && map_epoch > 0 &&
         map.resolution > 0 && !map.free.empty() && recent(stamp) &&
         recent(pose_sample) && recent(pose_received) && recent(depth_sample) &&
         recent(lidar_sample) && recent(received) && recent(depth_received) &&
         recent(lidar_received) && monotonic_recent(received_steady) &&
         monotonic_recent(depth_received_steady) &&
         monotonic_recent(lidar_received_steady);
}
bool MapSnapshotReceiver::changes_since(int64_t previous, Keys &changed) const {
  changed.clear();
  if (previous < delta_floor_ || previous > observation_revision)
    return false;
  for (const auto &delta : deltas_)
    if (delta.revision > previous)
      changed.insert(delta.changed.begin(), delta.changed.end());
  return true;
}
} // namespace powerplant
