#include "px4_powerplant_mission_cpp/topology.hpp"
#include "px4_powerplant_mission_cpp/planning_index.hpp"
#include <deque>
#include <numeric>
#include <set>

namespace powerplant {
namespace {
constexpr std::array<Key, 6> adjacent = {Key{1, 0, 0}, Key{-1, 0, 0},
                                        Key{0, 1, 0}, Key{0, -1, 0},
                                        Key{0, 0, 1}, Key{0, 0, -1}};
void validate_topology(const TopologyConfig &c) {
  for (double v : {c.core_extra_clearance_m, c.information_weight,
                   c.travel_time_weight, c.revisit_weight, c.switch_margin})
    if (!std::isfinite(v) || v < 0)
      throw std::invalid_argument("topology parameters must be finite and nonnegative");
}
std::vector<Key> sorted(const Keys &keys) {
  std::vector<Key> result(keys.begin(), keys.end());
  std::sort(result.begin(), result.end());
  return result;
}
struct InterfacePair { Key first, second; };
struct DisjointSet {
  std::vector<size_t> parent;
  explicit DisjointSet(size_t n) : parent(n) {
    std::iota(parent.begin(), parent.end(), 0);
  }
  size_t root(size_t n) {
    while (parent[n] != n) {
      parent[n] = parent[parent[n]];
      n = parent[n];
    }
    return n;
  }
  void join(size_t a, size_t b) {
    a = root(a); b = root(b);
    if (a != b) parent[std::max(a, b)] = std::min(a, b);
  }
};
} // namespace

uint64_t TopologySnapshot::region_at(const Key &k) const {
  const auto found = labels.find(k);
  return found == labels.end() ? 0 : found->second;
}
uint64_t TopologySnapshot::frontier_region(const Key &k) const {
  const auto found = frontier_regions.find(k);
  return found == frontier_regions.end() ? region_at(k) : found->second;
}
bool TopologySnapshot::matches(const PlanningMap &m, double clearance) const {
  return resolution == m.resolution && bounds == m.bounds &&
         clearance_radius_m == clearance;
}

std::shared_ptr<const TopologySnapshot> build_topology(
    const PlanningMap &m, double clearance_radius_m, int64_t epoch,
    int64_t revision, const TopologyConfig &config,
    std::shared_ptr<const TopologySnapshot> previous, const Cancel &cancel) {
  validate_topology(config);
  if (!std::isfinite(m.resolution) || m.resolution <= 0 ||
      !std::isfinite(clearance_radius_m) || clearance_radius_m < 0)
    throw std::invalid_argument("invalid topology map geometry");
  for (int axis = 0; axis < 3; ++axis)
    if (m.bounds[0][axis] > m.bounds[1][axis])
      throw std::invalid_argument("invalid topology bounds");
  const double lattice = std::ceil(config.core_extra_clearance_m / m.resolution);
  if (lattice > 16)
    throw std::invalid_argument("topology core footprint exceeds 16 voxels");
  check_cancel(cancel);
  auto index = m.index;
  if (!index || !index->matches(m.resolution, clearance_radius_m))
    index = PlanningIndex::update({}, m.free, m.occupied, {}, m.resolution,
                                  clearance_radius_m);
  const Keys safe = index->safe_in(m.bounds);
  const auto ordered = sorted(safe);
  auto out = std::make_shared<TopologySnapshot>();
  out->map_epoch = epoch;
  out->observation_revision = revision;
  out->resolution = m.resolution;
  out->bounds = m.bounds;
  out->clearance_radius_m = clearance_radius_m;
  out->configuration = config;
  // Even an incompatible graph configuration shares the epoch's ID space.
  // Allocate fresh identities instead of aliasing retained/revisit history;
  // only the compatible overlap matching below may reuse an existing ID.
  if (previous && previous->map_epoch == epoch)
    out->next_region_id = previous->next_region_id;
  if (safe.empty()) return out;

  std::vector<Key> footprint;
  const int n = int(lattice);
  for (int x = -n; x <= n; ++x)
    for (int y = -n; y <= n; ++y)
      for (int z = -n; z <= n; ++z)
        if ((double(x) * x + double(y) * y + double(z) * z) *
                m.resolution * m.resolution <=
            config.core_extra_clearance_m * config.core_extra_clearance_m + 1e-12)
          footprint.push_back({x, y, z});
  Keys cores;
  for (const auto &k : ordered) {
    check_cancel(cancel);
    bool core = true;
    for (const auto &offset : footprint)
      if (!safe.count(add(k, offset))) { core = false; break; }
    if (core) cores.insert(k);
  }
  KeyMap<size_t> raw;
  std::map<size_t, std::vector<Key>> seeds;
  size_t next_raw = 1;
  for (const auto &start : ordered) {
    if (!cores.count(start) || raw.count(start)) continue;
    const size_t id = next_raw++;
    std::deque<Key> queue{start};
    raw[start] = id;
    while (!queue.empty()) {
      check_cancel(cancel);
      const Key k = queue.front(); queue.pop_front();
      seeds[id].push_back(k);
      for (const auto &offset : adjacent) {
        const Key neighbor = add(k, offset);
        if (cores.count(neighbor) && !raw.count(neighbor)) {
          raw[neighbor] = id;
          queue.push_back(neighbor);
        }
      }
    }
  }
  // Every disconnected thin component gets a seed even when erosion removes
  // all its cells. It remains usable as a single conservative region.
  Keys component_seen;
  for (const auto &start : ordered) {
    if (!component_seen.insert(start).second) continue;
    std::deque<Key> queue{start};
    bool has_core = false;
    while (!queue.empty()) {
      check_cancel(cancel);
      const Key k = queue.front(); queue.pop_front();
      has_core = has_core || cores.count(k);
      for (const auto &offset : adjacent) {
        const Key neighbor = add(k, offset);
        if (safe.count(neighbor) && component_seen.insert(neighbor).second)
          queue.push_back(neighbor);
      }
    }
    if (!has_core) {
      raw[start] = next_raw;
      seeds[next_raw++].push_back(start);
    }
  }
  std::deque<Key> grow;
  for (auto &[id, cells] : seeds) {
    std::sort(cells.begin(), cells.end());
    for (const auto &k : cells) grow.push_back(k);
  }
  while (!grow.empty()) {
    check_cancel(cancel);
    const Key k = grow.front(); grow.pop_front();
    for (const auto &offset : adjacent) {
      const Key neighbor = add(k, offset);
      if (safe.count(neighbor) && !raw.count(neighbor)) {
        raw[neighbor] = raw.at(k);
        grow.push_back(neighbor);
      }
    }
  }

  std::map<size_t, uint64_t> stable;
  if (previous && previous->map_epoch == epoch &&
      previous->matches(m, clearance_radius_m) &&
      previous->configuration.core_extra_clearance_m == config.core_extra_clearance_m) {
    out->next_region_id = previous->next_region_id;
    std::map<std::pair<size_t, uint64_t>, size_t> overlap;
    for (const auto &[k, id] : raw) {
      const uint64_t old = previous->region_at(k);
      if (old) ++overlap[{id, old}];
    }
    struct Match { size_t count, raw; uint64_t old; };
    std::vector<Match> matches;
    for (const auto &[pair, count] : overlap)
      matches.push_back({count, pair.first, pair.second});
    std::sort(matches.begin(), matches.end(), [](const Match &a, const Match &b) {
      if (a.count != b.count) return a.count > b.count;
      return std::pair(a.old, a.raw) < std::pair(b.old, b.raw);
    });
    std::set<uint64_t> used;
    for (const auto &match : matches)
      if (!stable.count(match.raw) && used.insert(match.old).second)
        stable[match.raw] = match.old;
  }
  for (const auto &[id, cells] : seeds) {
    (void)cells;
    if (!stable.count(id)) {
      if (out->next_region_id == std::numeric_limits<uint64_t>::max())
        throw std::overflow_error("topology region id exhausted");
      stable[id] = out->next_region_id++;
    }
  }
  std::map<size_t, Vec3> sums;
  std::map<size_t, size_t> counts;
  for (const auto &k : ordered) {
    const size_t id = raw.at(k);
    out->labels[k] = stable.at(id);
    if (!counts.count(id)) sums[id] = Vec3::Zero();
    sums[id] += center(k, m.resolution);
    ++counts[id];
  }
  for (const auto &[id, cells] : seeds) {
    const Vec3 mean = sums.at(id) / double(counts.at(id));
    Key representative = cells.front();
    double best = inf;
    for (const auto &k : cells) {
      const double distance = (center(k, m.resolution) - mean).squaredNorm();
      if (distance < best) { best = distance; representative = k; }
    }
    const uint64_t stable_id = stable.at(id);
    out->regions.emplace(stable_id,
        TopologyRegion{stable_id, counts.at(id), representative, {}});
  }

  // Frontiers sit outside inflated safe centers. Associate their information
  // by observed-free propagation; this cannot create a traversable portal.
  KeyMap<uint64_t> observed_labels = out->labels;
  std::vector<std::pair<uint64_t, Key>> observed_seeds;
  observed_seeds.reserve(ordered.size());
  for (const auto &k : ordered)
    observed_seeds.emplace_back(out->labels.at(k), k);
  std::sort(observed_seeds.begin(), observed_seeds.end());
  for (const auto &[id, k] : observed_seeds) { (void)id; grow.push_back(k); }
  while (!grow.empty()) {
    check_cancel(cancel);
    const Key k = grow.front(); grow.pop_front();
    for (const auto &offset : adjacent) {
      const Key neighbor = add(k, offset);
      if (inside(neighbor, m.bounds) && m.free.count(neighbor) &&
          !m.occupied.count(neighbor) && !observed_labels.count(neighbor)) {
        observed_labels[neighbor] = observed_labels.at(k);
        grow.push_back(neighbor);
      }
    }
  }
  for (const auto &[k, id] : observed_labels) {
    check_cancel(cancel);
    for (const auto &offset : adjacent) {
      const Key neighbor = add(k, offset);
      if (inside(neighbor, m.bounds) && !m.free.count(neighbor) &&
          !m.occupied.count(neighbor)) {
        out->frontier_regions[k] = id;
        out->regions.at(id).unknown_neighbors.insert(neighbor);
      }
    }
  }

  // Graph distance from S's boundary selects a broad interface representative;
  // it is a ranking proxy, never a physical collision clearance certificate.
  KeyMap<int> depth;
  for (const auto &k : ordered)
    for (const auto &offset : adjacent)
      if (!safe.count(add(k, offset))) {
        depth[k] = 0; grow.push_back(k); break;
      }
  while (!grow.empty()) {
    check_cancel(cancel);
    const Key k = grow.front(); grow.pop_front();
    for (const auto &offset : adjacent) {
      const Key neighbor = add(k, offset);
      if (safe.count(neighbor) && !depth.count(neighbor)) {
        depth[neighbor] = depth.at(k) + 1; grow.push_back(neighbor);
      }
    }
  }
  std::map<std::pair<uint64_t, uint64_t>, std::vector<InterfacePair>> interfaces;
  for (const auto &k : ordered) {
    check_cancel(cancel);
    for (int axis = 0; axis < 3; ++axis) {
      Key neighbor = k; ++neighbor[axis];
      if (!safe.count(neighbor)) continue;
      uint64_t first = out->region_at(k), second = out->region_at(neighbor);
      if (first == second) continue;
      if (first < second) interfaces[{first, second}].push_back({k, neighbor});
      else interfaces[{second, first}].push_back({neighbor, k});
    }
  }
  for (const auto &[regions, pairs] : interfaces) {
    check_cancel(cancel);
    DisjointSet sets(pairs.size());
    KeyMap<std::vector<size_t>> touching;
    for (size_t i = 0; i < pairs.size(); ++i) {
      for (const Key k : {pairs[i].first, pairs[i].second}) {
        auto join_at = [&](const Key &q) {
          auto found = touching.find(q);
          if (found != touching.end())
            for (const size_t j : found->second) sets.join(i, j);
        };
        join_at(k);
        for (const auto &offset : adjacent) join_at(add(k, offset));
      }
      touching[pairs[i].first].push_back(i);
      touching[pairs[i].second].push_back(i);
    }
    std::map<size_t, std::vector<size_t>> components;
    for (size_t i = 0; i < pairs.size(); ++i) components[sets.root(i)].push_back(i);
    for (const auto &[component, members] : components) {
      (void)component;
      auto chosen = pairs[members.front()];
      int best_depth = -1;
      for (const size_t i : members) {
        const auto &pair = pairs[i];
        const int candidate = std::min(depth.at(pair.first), depth.at(pair.second));
        if (candidate > best_depth ||
            (candidate == best_depth && std::pair(pair.first, pair.second) <
                                           std::pair(chosen.first, chosen.second))) {
          chosen = pair; best_depth = candidate;
        }
      }
      out->portals.push_back({regions.first, regions.second, chosen.first,
                              chosen.second, members.size()});
    }
  }
  return out;
}

std::vector<RegionPriority> rank_topology_regions(
    const TopologyGuide &guide, double remaining,
    const std::function<std::optional<double>(uint64_t)> &travel_time,
    const std::function<bool(const Key &)> &eligible_unknown) {
  validate_topology(guide.configuration);
  if (!guide.snapshot || !travel_time || !std::isfinite(remaining) || remaining <= 0)
    return {};
  struct Candidate { uint64_t id; size_t unknown; double time; };
  std::vector<Candidate> candidates;
  size_t maximum_unknown = 0;
  for (const auto &[id, region] : guide.snapshot->regions) {
    const size_t unknown = eligible_unknown
        ? size_t(std::count_if(region.unknown_neighbors.begin(),
                              region.unknown_neighbors.end(), eligible_unknown))
        : region.unknown_neighbors.size();
    if (!unknown) continue;
    const auto time = travel_time(id);
    if (!time || !std::isfinite(*time) || *time < 0) continue;
    candidates.push_back({id, unknown, *time});
    maximum_unknown = std::max(maximum_unknown, unknown);
  }
  std::vector<RegionPriority> result;
  const auto &config = guide.configuration;
  for (const auto &candidate : candidates) {
    const double gain = std::log1p(candidate.unknown) / std::log1p(maximum_unknown);
    const double time = std::min(1.0, candidate.time / remaining);
    const auto found = guide.recent_observations.find(candidate.id);
    const double revisits = found == guide.recent_observations.end()
                                ? 0 : std::min(1.0, double(found->second) / 3);
    result.push_back({candidate.id, config.information_weight * gain -
                                    config.travel_time_weight * time -
                                    config.revisit_weight * revisits});
  }
  std::sort(result.begin(), result.end(), [](const auto &a, const auto &b) {
    return a.score != b.score ? a.score > b.score : a.region_id < b.region_id;
  });
  const auto retained = std::find_if(result.begin(), result.end(), [&](const auto &p) {
    return p.region_id == guide.retained_region_id;
  });
  if (retained != result.end() && !result.empty() &&
      result.front().score <= retained->score + config.switch_margin) {
    const auto selected = *retained;
    result.erase(retained);
    result.insert(result.begin(), selected);
  }
  return result;
}
} // namespace powerplant
