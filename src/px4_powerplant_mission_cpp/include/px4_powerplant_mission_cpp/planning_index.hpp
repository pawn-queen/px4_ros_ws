#pragma once

#include "px4_powerplant_mission_cpp/core.hpp"
#include <memory>

namespace powerplant {

struct PlanningIndexStats {
  size_t changed_voxels = 0;
  size_t frontier_recomputed = 0;
  size_t safety_recomputed = 0;
  size_t copied_shards = 0;
  size_t frontier_count = 0;
  size_t safe_count = 0;
  bool full_rebuild = false;
};

// Immutable planning data shared with worker snapshots. Updates copy only the
// hash shards touched by changed cells and their finite dependency footprint.
// Bounds are deliberately a query constraint, never a persistent cache key.
class PlanningIndex {
public:
  static std::shared_ptr<const PlanningIndex>
  update(const std::shared_ptr<const PlanningIndex> &previous, const Keys &free,
         const Keys &occupied, const Keys &changed, double resolution,
         double clearance_radius);

  bool matches(double resolution, double clearance_radius) const;
  bool matches_resolution(double resolution) const { return resolution == resolution_; }
  bool is_safe(const Key &key, const Bounds &bounds) const;
  Keys safe_in(const Bounds &bounds) const;
  KeyMap<Keys> frontiers_in(const Bounds &bounds) const;
  const PlanningIndexStats &stats() const { return stats_; }

private:
  PlanningIndex() = default;
  struct Shard {
    KeyMap<Keys> frontiers;
    Keys safe;
  };
  static constexpr size_t shard_count = 256;
  std::array<std::shared_ptr<const Shard>, shard_count> shards_{};
  double resolution_ = 0, clearance_radius_ = 0;
  std::vector<Key> offsets_;
  Key offset_extent_{};
  PlanningIndexStats stats_;
  bool footprint_inside(const Key &, const Bounds &) const;
};

} // namespace powerplant
