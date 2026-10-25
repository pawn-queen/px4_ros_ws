#include "px4_powerplant_mission_cpp/core.hpp"
#include "px4_powerplant_mission_cpp/raycast.hpp"

namespace powerplant {
void VoxelGrid::update(const Key &k, double delta) {
  auto [it, inserted] = cells.emplace(k, 0);
  if (inserted)
    order.push_back(k);
  it->second = std::clamp(it->second + delta, minimum, maximum);
}
void VoxelGrid::clear() {
  cells.clear();
  order.clear();
  ++revision;
}
size_t VoxelGrid::insert_points(const Path &points) {
  size_t n = 0;
  for (const auto &p : points)
    if (p.allFinite()) {
      update(key(p, resolution), hit);
      ++n;
    }
  if (n)
    ++revision;
  return n;
}
size_t VoxelGrid::insert_rays(const Path &origins, const Path &points) {
  if (origins.size() != points.size())
    throw std::invalid_argument("ray origin count mismatch");
  if (!std::isfinite(resolution) || resolution <= 0)
    throw std::invalid_argument("voxel resolution must be finite and positive");
  Keys hits, free;
  for (size_t i = 0; i < points.size(); ++i) {
    if (!origins[i].allFinite() || !points[i].allFinite())
      continue;
    const Key hk = key(points[i], resolution);
    hits.insert(hk);
    walk_ray_positive(origins[i], points[i], resolution,
                      [&](const Key &cell, long double, long double) {
                        free.insert(cell);
                      });
  }
  for (const auto &k : free)
    if (!hits.count(k))
      update(k, miss);
  for (const auto &k : hits)
    update(k, hit);
  if (!hits.empty())
    ++revision;
  return hits.size();
}
VoxelSnapshot VoxelGrid::snapshot(size_t limit) const {
  VoxelSnapshot s;
  s.observed = cells.size();
  s.complete = s.observed <= limit;
  s.points.reserve(std::min(limit, s.observed));
  for (const auto &k : order) {
    double value = cells.at(k);
    int state = -1;
    if (value >= occupied_threshold) {
      state = 1;
      ++s.occupied;
    } else if (value <= free_threshold) {
      state = 0;
      ++s.free;
    } else
      ++s.uncertain;
    if (!s.bounds)
      s.bounds = Bounds{k, k};
    else
      for (int i = 0; i < 3; ++i) {
        (*s.bounds)[0][i] = std::min((*s.bounds)[0][i], k[i]);
        (*s.bounds)[1][i] = std::max((*s.bounds)[1][i], k[i]);
      }
    if (s.points.size() < limit) {
      auto p = center(k, resolution);
      s.points.push_back(
          {float(p.x()), float(p.y()), float(p.z()), float(state)});
    }
  }
  return s;
}
Path VoxelGrid::occupied_points(size_t limit) const {
  std::vector<Key> keys;
  for (const auto &k : order)
    if (cells.at(k) >= occupied_threshold)
      keys.push_back(k);
  Path points;
  if (!limit)
    return points;
  const size_t step = std::max(size_t(1), keys.size() / limit);
  for (size_t i = 0; i < keys.size() && points.size() < limit; i += step)
    points.push_back(center(keys[i], resolution));
  return points;
}
std::optional<Grid2D>
VoxelGrid::project(double zmin, double zmax,
                   std::optional<std::array<double, 4>> bounds,
                   double padding) const {
  Path free, occupied;
  for (const auto &k : order) {
    auto p = center(k, resolution);
    if (p.z() < zmin || p.z() > zmax)
      continue;
    double value = cells.at(k);
    if (value >= occupied_threshold)
      occupied.push_back(p);
    else if (value <= free_threshold)
      free.push_back(p);
  }
  double xmin = inf, ymin = inf, xmax = -inf, ymax = -inf;
  if (bounds) {
    xmin = (*bounds)[0];
    ymin = (*bounds)[1];
    xmax = (*bounds)[2];
    ymax = (*bounds)[3];
  } else {
    if (free.empty() && occupied.empty())
      return {};
    for (auto points : {&free, &occupied})
      for (const auto &p : *points) {
        xmin = std::min(xmin, p.x());
        ymin = std::min(ymin, p.y());
        xmax = std::max(xmax, p.x());
        ymax = std::max(ymax, p.y());
      }
    xmin = std::floor((xmin - padding) / resolution) * resolution;
    ymin = std::floor((ymin - padding) / resolution) * resolution;
    xmax = std::ceil((xmax + padding) / resolution) * resolution;
    ymax = std::ceil((ymax + padding) / resolution) * resolution;
  }
  if (!std::isfinite(xmin) || !std::isfinite(ymin) || !std::isfinite(xmax) ||
      !std::isfinite(ymax) || xmax <= xmin || ymax <= ymin)
    return {};
  Grid2D g;
  g.x = xmin;
  g.y = ymin;
  g.resolution = resolution;
  g.width = std::max(1, int(std::ceil((xmax - xmin) / resolution)));
  g.height = std::max(1, int(std::ceil((ymax - ymin) / resolution)));
  g.data.assign(size_t(g.width) * g.height, -1);
  for (auto points : {&free, &occupied})
    for (const auto &p : *points) {
      int x = int((p.x() - xmin) / resolution),
          y = int((p.y() - ymin) / resolution);
      if (x >= 0 && x < g.width && y >= 0 && y < g.height)
        g.data[y * g.width + x] = points == &free ? 0 : 100;
    }
  return g;
}
bool PoseHistory::add(const StampedPose &p) {
  bool jumped = !samples.empty() && p.stamp < samples.back().stamp - 250000000;
  if (jumped)
    samples.clear();
  auto it =
      std::lower_bound(samples.begin(), samples.end(), p.stamp,
                       [](const auto &a, int64_t t) { return a.stamp < t; });
  if (it != samples.end() && it->stamp == p.stamp)
    *it = p;
  else
    samples.insert(it, p);
  int64_t cutoff = samples.back().stamp - 2000000000;
  samples.erase(
      samples.begin(),
      std::lower_bound(samples.begin(), samples.end(), cutoff,
                       [](const auto &a, int64_t t) { return a.stamp < t; }));
  return jumped;
}
std::optional<StampedPose> PoseHistory::at(int64_t stamp) const {
  auto it =
      std::lower_bound(samples.begin(), samples.end(), stamp,
                       [](const auto &a, int64_t t) { return a.stamp < t; });
  if (it != samples.end() && it->stamp == stamp)
    return *it;
  if (it == samples.begin() || it == samples.end())
    return {};
  const auto &before = *(it - 1);
  if (it->stamp - before.stamp > 250000000)
    return {};
  double f = double(stamp - before.stamp) / double(it->stamp - before.stamp);
  Quat q1 = it->orientation, q0 = before.orientation;
  double dot = q0.dot(q1);
  if (dot < 0) {
    q1.coeffs() *= -1;
    dot = -dot;
  }
  Quat q;
  if (dot > .9995)
    q.coeffs() = q0.coeffs() + f * (q1.coeffs() - q0.coeffs());
  else
    q = q0.slerp(f, q1);
  return StampedPose{stamp,
                     before.position + f * (it->position - before.position),
                     q.normalized()};
}
bool YawScan::update(double actual, int64_t now, bool ready, double tolerance) {
  if (index >= steps)
    return true;
  if (!ready || std::abs(wrap_pi(target() - actual)) > tolerance) {
    reached_since = 0;
    return false;
  }
  if (!reached_since) {
    reached_since = now;
    return false;
  }
  if ((now - reached_since) * 1e-9 >= dwell) {
    ++index;
    reached_since = 0;
  }
  return index >= steps;
}
std::pair<Vec3, double> SlewState::update(const Vec3 &target, double y,
                                          int64_t now, double horizontal,
                                          double vertical, double yaw_rate) {
  if (!position)
    reset(target, y, now);
  double dt = (now - stamp) * 1e-9;
  if (dt <= 0 || dt > 1)
    dt = .1;
  stamp = now;
  Vec3 delta = target - *position;
  double xy = delta.head<2>().norm(), step = horizontal * dt;
  if (step > 0 && xy > step)
    position->head<2>() += delta.head<2>() * (step / xy);
  else
    position->head<2>() = target.head<2>();
  step = vertical * dt;
  (*position).z() += step > 0 && std::abs(delta.z()) > step
                         ? std::copysign(step, delta.z())
                         : delta.z();
  double dy = wrap_pi(y - heading);
  step = yaw_rate * dt;
  heading =
      wrap_pi(heading +
              (step > 0 && std::abs(dy) > step ? std::copysign(step, dy) : dy));
  return {*position, heading};
}
} // namespace powerplant
