#include "px4_powerplant_mission_cpp/map_snapshot.hpp"
namespace powerplant {
bool MapSnapshotReceiver::update_status(const std::string &text, int64_t now) {
  auto p = parse(text);
  if (!p["depth_fused_count"].isInt64() || !p["lidar_fused_count"].isInt64())
    throw std::invalid_argument("invalid fusion count");
  auto dc = p["depth_fused_count"].asInt64(),
       lc = p["lidar_fused_count"].asInt64();
  if (dc < 0 || lc < 0)
    throw std::invalid_argument("negative fusion count");
  bool restarted = status_received && (dc < depth_count || lc < lidar_count);
  if (dc > depth_count || (restarted && dc > 0))
    depth_received = now;
  if (lc > lidar_count || (restarted && lc > 0))
    lidar_received = now;
  depth_count = dc;
  lidar_count = lc;
  depth_sample = p.get("depth_last_sample_stamp_ns", JsonValue(0)).asInt64();
  lidar_sample = p.get("lidar_last_sample_stamp_ns", JsonValue(0)).asInt64();
  status_received = true;
  if (restarted) {
    complete = false;
    revision = -1;
    map.free.clear();
    map.occupied.clear();
  }
  status = p;
  if (!p.get("observed_voxels_complete", false).asBool() ||
      p["map_stamp_ns"].asInt64() != stamp)
    complete = false;
  return restarted;
}
bool MapSnapshotReceiver::commit(const std::string &frame, int64_t now) {
  if (!cloud || !status)
    return false;
  const auto &p = *status;
  int64_t ts = stamp_ns(cloud->header.stamp);
  if (!p["map_stamp_ns"].isInt64() || ts != p["map_stamp_ns"].asInt64() ||
      cloud->header.frame_id != frame)
    return false;
  if (!p["observed_voxels_complete"].asBool() ||
      !p["mapping_enabled"].asBool()) {
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
      expected < 0 || expected != p["observed_voxels_point_count"].asInt64())
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
  if (free != map.free || occupied != map.occupied)
    ++observation_revision;
  map.free = std::move(free);
  map.occupied = std::move(occupied);
  map.resolution = resolution;
  revision = rev;
  stamp = ts;
  received = now;
  complete = true;
  cloud.reset();
  status.reset();
  return true;
}
bool MapSnapshotReceiver::fresh(int64_t now, double max_age) const {
  auto recent = [&](int64_t t) {
    return t > 0 && now >= t && (now - t) * 1e-9 <= max_age;
  };
  return complete && map.resolution > 0 && !map.free.empty() &&
         recent(received) && recent(depth_received) && recent(lidar_received);
}
} // namespace powerplant
