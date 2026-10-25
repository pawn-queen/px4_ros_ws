#include "px4_powerplant_mission_cpp/planning_index.hpp"

namespace powerplant {
namespace {
constexpr std::array<Key, 6> neighbors = {Key{1, 0, 0}, Key{-1, 0, 0},
                                          Key{0, 1, 0}, Key{0, -1, 0},
                                          Key{0, 0, 1}, Key{0, 0, -1}};

std::optional<Key> checked_add(const Key &k, const Key &offset) {
  Key result;
  for (int axis = 0; axis < 3; ++axis) {
    const int64_t value = int64_t(k[axis]) + offset[axis];
    if (value < std::numeric_limits<int>::min() ||
        value > std::numeric_limits<int>::max())
      return {};
    result[axis] = static_cast<int>(value);
  }
  return result;
}

bool known_free(const Key &k, const Keys &free, const Keys &occupied) {
  return free.count(k) && !occupied.count(k);
}
} // namespace

bool PlanningIndex::matches(double resolution, double clearance_radius) const {
  return resolution == resolution_ && clearance_radius == clearance_radius_;
}

std::shared_ptr<const PlanningIndex>
PlanningIndex::update(const std::shared_ptr<const PlanningIndex> &previous,
                      const Keys &free, const Keys &occupied,
                      const Keys &changed, double resolution,
                      double clearance_radius) {
  if (!std::isfinite(resolution) || resolution <= 0 ||
      !std::isfinite(clearance_radius) || clearance_radius < 0)
    throw std::invalid_argument(
        "invalid planning index resolution or clearance");
  const bool rebuild =
      !previous || !previous->matches(resolution, clearance_radius);
  auto next =
      rebuild ? std::shared_ptr<PlanningIndex>(new PlanningIndex)
              : std::shared_ptr<PlanningIndex>(new PlanningIndex(*previous));
  next->stats_ = {};
  next->stats_.full_rebuild = rebuild;
  next->stats_.changed_voxels = changed.size();
  if (rebuild) {
    next->resolution_ = resolution;
    next->clearance_radius_ = clearance_radius;
    // Protect against unbounded configuration work. Reject rather than use a
    // smaller footprint, which would silently weaken the clearance guarantee.
    const double lattice_radius = std::ceil(clearance_radius / resolution);
    if (lattice_radius > 64)
      throw std::invalid_argument(
          "planning clearance footprint exceeds 64 voxels");
    const int n = static_cast<int>(lattice_radius);
    for (int x = -n; x <= n; ++x)
      for (int y = -n; y <= n; ++y)
        for (int z = -n; z <= n; ++z)
          if ((double(x) * x + double(y) * y + double(z) * z) * resolution *
                  resolution <=
              clearance_radius * clearance_radius + 1e-12) {
            next->offsets_.push_back({x, y, z});
            next->offset_extent_[0] =
                std::max(next->offset_extent_[0], std::abs(x));
            next->offset_extent_[1] =
                std::max(next->offset_extent_[1], std::abs(y));
            next->offset_extent_[2] =
                std::max(next->offset_extent_[2], std::abs(z));
          }
    for (auto &shard : next->shards_)
      shard = std::make_shared<Shard>();
  } else {
    next->stats_.frontier_count = previous->stats_.frontier_count;
    next->stats_.safe_count = previous->stats_.safe_count;
  }

  Keys frontier_dirty, safety_dirty;
  if (rebuild) {
    // Occupied wins if a caller supplies overlapping classifications.
    frontier_dirty = free;
    safety_dirty = free;
  } else {
    frontier_dirty.reserve(changed.size() * 7);
    for (const auto &k : changed) {
      frontier_dirty.insert(k);
      for (const auto &offset : neighbors)
        if (auto neighbor = checked_add(k, offset))
          frontier_dirty.insert(*neighbor);
      // A center depends on cell k iff center + offset == k. The spherical
      // footprint is symmetric, hence k + offset enumerates these centers.
      for (const auto &offset : next->offsets_)
        if (auto candidate = checked_add(k, offset))
          safety_dirty.insert(*candidate);
    }
  }
  next->stats_.frontier_recomputed = frontier_dirty.size();
  next->stats_.safety_recomputed = safety_dirty.size();

  std::array<std::shared_ptr<Shard>, shard_count> writable{};
  auto write_shard = [&](const Key &k) -> Shard & {
    const size_t bucket = KeyHash{}(k) % shard_count;
    if (!writable[bucket]) {
      writable[bucket] = std::make_shared<Shard>(*next->shards_[bucket]);
      next->shards_[bucket] = writable[bucket];
      ++next->stats_.copied_shards;
    }
    return *writable[bucket];
  };

  for (const auto &k : frontier_dirty) {
    Keys unknown;
    if (known_free(k, free, occupied))
      for (const auto &offset : neighbors)
        if (auto neighbor = checked_add(k, offset))
          if (!free.count(*neighbor) && !occupied.count(*neighbor))
            unknown.insert(*neighbor);
    const size_t bucket = KeyHash{}(k) % shard_count;
    const auto &old = next->shards_[bucket]->frontiers;
    const auto found = old.find(k);
    if ((unknown.empty() && found == old.end()) ||
        (found != old.end() && found->second == unknown))
      continue;
    auto &frontiers = write_shard(k).frontiers;
    if (unknown.empty()) {
      frontiers.erase(k);
      --next->stats_.frontier_count;
    } else {
      if (!frontiers.count(k))
        ++next->stats_.frontier_count;
      frontiers[k] = std::move(unknown);
    }
  }
  for (const auto &k : safety_dirty) {
    bool safe = known_free(k, free, occupied);
    if (safe)
      for (const auto &offset : next->offsets_) {
        const auto neighbor = checked_add(k, offset);
        if (!neighbor || !known_free(*neighbor, free, occupied)) {
          safe = false;
          break;
        }
      }
    const size_t bucket = KeyHash{}(k) % shard_count;
    const bool was_safe = next->shards_[bucket]->safe.count(k);
    if (safe == was_safe)
      continue;
    auto &safe_keys = write_shard(k).safe;
    if (safe) {
      safe_keys.insert(k);
      ++next->stats_.safe_count;
    } else {
      safe_keys.erase(k);
      --next->stats_.safe_count;
    }
  }
  return next;
}

bool PlanningIndex::footprint_inside(const Key &k, const Bounds &bounds) const {
  for (int axis = 0; axis < 3; ++axis)
    if (int64_t(k[axis]) - offset_extent_[axis] < bounds[0][axis] ||
        int64_t(k[axis]) + offset_extent_[axis] > bounds[1][axis])
      return false;
  return true;
}

bool PlanningIndex::is_safe(const Key &k, const Bounds &bounds) const {
  return footprint_inside(k, bounds) &&
         shards_[KeyHash{}(k) % shard_count]->safe.count(k);
}

Keys PlanningIndex::safe_in(const Bounds &bounds) const {
  Keys result;
  result.reserve(stats_.safe_count);
  for (const auto &shard : shards_)
    for (const auto &k : shard->safe)
      if (footprint_inside(k, bounds))
        result.insert(k);
  return result;
}

KeyMap<Keys> PlanningIndex::frontiers_in(const Bounds &bounds) const {
  KeyMap<Keys> result;
  result.reserve(stats_.frontier_count);
  for (const auto &shard : shards_)
    for (const auto &[k, neighbors] : shard->frontiers) {
      if (!inside(k, bounds))
        continue;
      Keys unknown;
      for (const auto &neighbor : neighbors)
        if (inside(neighbor, bounds))
          unknown.insert(neighbor);
      if (!unknown.empty())
        result.emplace(k, std::move(unknown));
    }
  return result;
}

} // namespace powerplant
