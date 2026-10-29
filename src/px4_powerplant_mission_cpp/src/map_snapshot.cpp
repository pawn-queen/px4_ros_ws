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
        !p["mapping_enabled"].isBool() ||
        !p["observed_voxels_complete"].isBool() ||
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
    // Transfer old containers in O(1); async consumers return this holder to
    // the decoder for destruction, including the potentially large delta log.
    auto retired = std::make_shared<MapSnapshotReceiver>();
    std::swap(retired->map, map);
    std::swap(retired->observed_bounds, observed_bounds);
    std::swap(retired->deltas_, deltas_);
    map.resolution = retired->map.resolution;
    retired_geometry_.push_back(std::move(retired));
    committed_evidence_.reset();
    committed_status_sequence_ = 0;
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
  if ((status_received && !restarted &&
       (dc < depth_count || lc < lidar_count ||
        (stamp > 0 && p["map_stamp_ns"].asInt64() < stamp) ||
        (p["map_revision"].isInt64() && p["map_revision"].asInt64() < revision))) ||
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
      !p["mapping_enabled"].asBool())
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
  std::optional<Bounds> decoded_bounds;
  for (const auto &row : rows) {
    Vec3 v(row[0], row[1], row[2]);
    float state = row[3];
    if (!v.allFinite() || (state != -1 && state != 0 && state != 1))
      return false;
    auto k = key(v, resolution);
    if (!observed.insert(k).second)
      return false;
    if (!decoded_bounds)
      decoded_bounds = Bounds{k, k};
    else
      for (int axis = 0; axis < 3; ++axis) {
        (*decoded_bounds)[0][axis] = std::min((*decoded_bounds)[0][axis], k[axis]);
        (*decoded_bounds)[1][axis] = std::max((*decoded_bounds)[1][axis], k[axis]);
      }
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
  observed_bounds = decoded_bounds;
  revision = rev;
  stamp = ts;
  received = now;
  received_steady = monotonic_now;
  committed_evidence_ = MapSnapshotEvidence{
      pose_sample, pose_received, depth_sample, lidar_sample, depth_received,
      lidar_received, depth_received_steady, lidar_received_steady};
  committed_status_sequence_ = status_sequence;
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
  // New status may report later sensor samples while geometry is still in the
  // decoder. Only evidence captured with the committed geometry refreshes it.
  const auto evidence = committed_evidence_.value_or(MapSnapshotEvidence{
      pose_sample, pose_received, depth_sample, lidar_sample, depth_received,
      lidar_received, depth_received_steady, lidar_received_steady});
  return complete && localization_healthy && map_epoch > 0 &&
         map.resolution > 0 && !map.free.empty() && recent(stamp) &&
         recent(evidence.pose_sample) && recent(evidence.pose_received) &&
         recent(evidence.depth_sample) && recent(evidence.lidar_sample) &&
         recent(received) && recent(evidence.depth_received) &&
         recent(evidence.lidar_received) && monotonic_recent(received_steady) &&
         monotonic_recent(evidence.depth_received_steady) &&
         monotonic_recent(evidence.lidar_received_steady);
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

std::optional<MapCommitRequest> MapSnapshotReceiver::make_commit_request(
    const std::string &frame, int64_t now, int64_t monotonic_now) const {
  if (!cloud || !status || !localization_healthy) return {};
  const auto &p = *status;
  const int64_t sample = stamp_ns(cloud->header.stamp);
  if (!p["map_stamp_ns"].isInt64() || sample != p["map_stamp_ns"].asInt64() ||
      cloud->header.frame_id != frame || p["map_epoch"].asInt64() != map_epoch ||
      !p["mapping_enabled"].asBool() || !p["observed_voxels_complete"].asBool())
    return {};
  for (const char *name : {"map_revision", "observed_voxel_count",
                           "observed_voxels_point_count", "free_voxel_count",
                           "occupied_voxel_count", "uncertain_voxel_count"})
    if (!p[name].isInt64()) return {};
  const int64_t expected = p["observed_voxel_count"].asInt64();
  if (expected < 0 || p["observed_voxels_point_count"].asInt64() != expected ||
      uint64_t(cloud->width) * cloud->height != uint64_t(expected) ||
      p["map_revision"].asInt64() < revision || (stamp && sample < stamp))
    return {};
  MapCommitRequest request;
  request.cloud = cloud;
  request.status = p;
  request.frame = frame;
  request.clearance_radius_m = index_clearance_radius_m;
  request.received_ns = now;
  request.received_steady_ns = monotonic_now ? monotonic_now : steady_ns();
  request.source_epoch = map_epoch;
  request.source_sequence = status_sequence;
  request.source_stamp = sample;
  request.source_revision = p["map_revision"].asInt64();
  request.basis_observation_revision = observation_revision;
  request.evidence = MapSnapshotEvidence{
      pose_sample, pose_received, depth_sample, lidar_sample, depth_received,
      lidar_received, depth_received_steady, lidar_received_steady};
  return request;
}

bool MapSnapshotReceiver::adopt_geometry(MapSnapshotReceiver &decoded,
                                        int64_t epoch, int64_t sequence) {
  if (!decoded.complete || !localization_healthy || epoch != map_epoch ||
      decoded.map_epoch != epoch || decoded.status_sequence != sequence ||
      sequence > status_sequence || sequence < committed_status_sequence_ ||
      decoded.revision < revision || decoded.stamp < stamp ||
      decoded.observation_revision < observation_revision ||
      decoded.index_clearance_radius_m != index_clearance_radius_m)
    return false;
  if (status && (!(*status)["mapping_enabled"].asBool() ||
                 !(*status)["observed_voxels_complete"].asBool() ||
                 !(*status)["localization_healthy"].asBool()))
    return false;
  std::swap(map, decoded.map);
  std::swap(observed_bounds, decoded.observed_bounds);
  std::swap(deltas_, decoded.deltas_);
  std::swap(delta_cells_, decoded.delta_cells_);
  std::swap(delta_floor_, decoded.delta_floor_);
  std::swap(committed_evidence_, decoded.committed_evidence_);
  revision = decoded.revision;
  observation_revision = decoded.observation_revision;
  stamp = decoded.stamp;
  received = decoded.received;
  received_steady = decoded.received_steady;
  committed_status_sequence_ = sequence;
  complete = true;
  return true;
}

std::vector<std::shared_ptr<MapSnapshotReceiver>>
MapSnapshotReceiver::take_retired_geometry() {
  std::vector<std::shared_ptr<MapSnapshotReceiver>> retired;
  retired.swap(retired_geometry_);
  return retired;
}

AsyncMapCommitter::AsyncMapCommitter() : thread_([this] { run(); }) {}
AsyncMapCommitter::~AsyncMapCommitter() {
  {
    std::lock_guard<std::mutex> lock(mutex_);
    stop_ = true;
    ++generation_;
  }
  ready_.notify_one();
  thread_.join();
}
void AsyncMapCommitter::submit(MapCommitRequest request) {
  {
    std::lock_guard<std::mutex> lock(mutex_);
    // New publications replace only the pending job. The currently decoding
    // valid snapshot may still be adopted; otherwise sustained input could
    // indefinitely cancel every large-map result before it reaches the writer.
    pending_ = Pending{generation_, std::move(request)};
  }
  ready_.notify_one();
}
std::optional<MapCommitResult> AsyncMapCommitter::take() {
  std::lock_guard<std::mutex> lock(mutex_);
  auto result = std::move(result_);
  result_.reset();
  return result;
}
void AsyncMapCommitter::retire(std::shared_ptr<MapSnapshotReceiver> geometry) {
  if (!geometry) return;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    retired_.push_back(std::move(geometry));
  }
  ready_.notify_one();
}
void AsyncMapCommitter::cancel() {
  {
    std::lock_guard<std::mutex> lock(mutex_);
    ++generation_;
    pending_.reset();
    if (result_ && result_->decoded) retired_.push_back(std::move(result_->decoded));
    result_.reset();
  }
  ready_.notify_one();
}
void AsyncMapCommitter::run() {
  for (;;) {
    std::optional<Pending> pending;
    std::vector<std::shared_ptr<MapSnapshotReceiver>> retired;
    {
      std::unique_lock<std::mutex> lock(mutex_);
      ready_.wait(lock, [this] { return stop_ || pending_ || !retired_.empty(); });
      retired.swap(retired_);
      if (stop_) {
        lock.unlock();
        retired.clear();
        return;
      }
      pending = std::move(pending_);
      pending_.reset();
    }
    retired.clear(); // Large hash containers are destroyed off the ROS writer.
    if (!pending) continue;
    const auto &request = pending->request;
    MapCommitResult out;
    out.source_epoch = request.source_epoch;
    out.source_sequence = request.source_sequence;
    out.source_stamp = request.source_stamp;
    out.source_revision = request.source_revision;
    out.source_cloud = request.cloud;
    try {
      if (state_.observation_revision < request.basis_observation_revision) {
        // Epoch changes may have happened on the writer without paired clouds.
        // Losing the intermediate history must invalidate old ROI proofs.
        state_.observation_revision = request.basis_observation_revision;
        state_.deltas_.clear();
        state_.delta_cells_ = 0;
        state_.delta_floor_ = state_.observation_revision;
      }
      state_.update_status(serialize(request.status), request.received_ns,
                            request.received_steady_ns);
      state_.retired_geometry_.clear();
      if (state_.map_epoch != request.source_epoch ||
          state_.status_sequence != request.source_sequence)
        throw std::invalid_argument("decoder request epoch/sequence retired");
      // Rebuilding an index after a parameter change can legitimately reuse
      // the same immutable status/payload pair without refreshing its evidence.
      state_.status = request.status;
      state_.cloud = request.cloud;
      state_.index_clearance_radius_m = request.clearance_radius_m;
      state_.depth_received = request.evidence.depth_received;
      state_.lidar_received = request.evidence.lidar_received;
      state_.depth_received_steady = request.evidence.depth_received_steady;
      state_.lidar_received_steady = request.evidence.lidar_received_steady;
      out.success = state_.commit(request.frame, request.received_ns,
                                   request.received_steady_ns);
      if (out.success)
        out.decoded = std::make_shared<MapSnapshotReceiver>(state_);
      else
        out.error = "map fingerprint, payload, count or geometry validation failed";
    } catch (const std::exception &error) {
      out.success = false;
      out.error = error.what();
    }
    std::optional<MapCommitResult> replaced;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      if (!stop_ && pending->generation == generation_) {
        replaced.swap(result_);
        result_ = std::move(out);
      }
    }
    // Replaced/cancelled output containers also retire on this thread.
  }
}
} // namespace powerplant
