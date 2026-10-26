#pragma once
#include "px4_powerplant_mission_cpp/core.hpp"

namespace powerplant {
// These are guidance weights, not collision clearances or a semantic room model.
struct TopologyConfig {
  double core_extra_clearance_m = .5;
  double information_weight = .6, travel_time_weight = .3,
         revisit_weight = .1, switch_margin = .1;
};
struct TopologyRegion {
  uint64_t id = 0;
  size_t voxel_count = 0;
  Key representative{};
  Keys unknown_neighbors;
};
struct TopologyPortal {
  uint64_t first_region = 0, second_region = 0;
  Key first_cell{}, second_cell{};
  size_t interface_pairs = 0;
};
struct TopologySnapshot {
  int64_t map_epoch = 0, observation_revision = 0;
  double resolution = .25, clearance_radius_m = .35;
  Bounds bounds{};
  TopologyConfig configuration;
  uint64_t next_region_id = 1;
  KeyMap<uint64_t> labels;
  // Boundary free cells lack full flight clearance. Their information is
  // associated through observed-free space, never used to create portal edges.
  KeyMap<uint64_t> frontier_regions;
  std::map<uint64_t, TopologyRegion> regions;
  std::vector<TopologyPortal> portals;
  uint64_t region_at(const Key &k) const;
  uint64_t frontier_region(const Key &k) const;
  bool matches(const PlanningMap &, double clearance) const;
};
struct TopologyGuide {
  std::shared_ptr<const TopologySnapshot> snapshot;
  TopologyConfig configuration;
  uint64_t retained_region_id = 0;
  // Caller supplies only completed, qualified observations in its recent window.
  std::map<uint64_t, size_t> recent_observations;
};
struct RegionPriority {
  uint64_t region_id = 0;
  double score = 0;
};
std::shared_ptr<const TopologySnapshot> build_topology(
    const PlanningMap &, double clearance_radius_m, int64_t map_epoch,
    int64_t observation_revision, const TopologyConfig & = {},
    std::shared_ptr<const TopologySnapshot> previous = {},
    const Cancel & = {});
// travel_time receives a reachable region and returns a feasible time estimate;
// nullopt excludes it. Empty/no-frontier regions do not consume quotas.
std::vector<RegionPriority> rank_topology_regions(
    const TopologyGuide &, double remaining_flight_s,
    const std::function<std::optional<double>(uint64_t)> &travel_time);
} // namespace powerplant
