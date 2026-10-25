#pragma once
#include "px4_powerplant_mission_cpp/core.hpp"

namespace powerplant {
// FLU front-camera angular bounds, including an off-centre principal point.
// Pixel edges are used; the caller still supplies verified, registered
// intrinsics.
struct CameraViewGeometry {
  int width = 0, height = 0;
  double fx = 0, fy = 0, cx = 0, cy = 0;
  bool from_camera_info = false;
  std::string frame;
  int64_t info_stamp = 0;
  bool valid() const {
    return width > 0 && height > 0 && std::isfinite(fx) && fx > 0 &&
           std::isfinite(fy) && fy > 0 && std::isfinite(cx) &&
           std::isfinite(cy);
  }
  double yaw_min() const { return std::atan2(cx - (width - .5), fx); }
  double yaw_max() const { return std::atan2(cx + .5, fx); }
  double pitch_min() const { return std::atan2(cy - (height - .5), fy); }
  double pitch_max() const { return std::atan2(cy + .5, fy); }
};

class HeadingCoverage {
public:
  explicit HeadingCoverage(int bins = 72) { reset(bins); }
  void reset(int bins) {
    if (bins < 4 || bins > 720)
      throw std::invalid_argument("scan coverage bins must be in [4,720]");
    observed_.assign(size_t(bins), false);
    count_ = 0;
  }
  int bins() const { return int(observed_.size()); }
  int count() const { return count_; }
  double fraction() const { return double(count_) / bins(); }
  bool seen(int bin) const {
    return bin >= 0 && bin < bins() && observed_[size_t(bin)];
  }
  bool mark(int source_bin, int source_bins) {
    if (source_bins < 4 || source_bin < 0 || source_bin >= source_bins)
      return false;
    // A source sector only proves a target sector if its angular uncertainty
    // lies wholly within that target. Coarse metadata cannot fill fine bins.
    const double begin = double(source_bin) * bins() / source_bins;
    const double end = double(source_bin + 1) * bins() / source_bins;
    const int first = int(std::floor(begin + 1e-10));
    const int last = int(std::floor(end - 1e-10));
    if (first != last || first < 0 || first >= bins() || seen(first))
      return false;
    observed_[size_t(first)] = true;
    ++count_;
    return true;
  }
  // Positive-yaw search order avoids repeatedly staring at an invalid-depth
  // patch; the current target can be revisited after the other headings.
  double next_unseen(double current, double minimum_turn) const {
    const double step = 2 * pi / bins();
    double best_delta = inf, best = wrap_pi(current + minimum_turn);
    for (int bin = 0; bin < bins(); ++bin) {
      if (seen(bin))
        continue;
      const double heading = -pi + (bin + .5) * step;
      double delta = std::fmod(heading - current + 2 * pi, 2 * pi);
      if (delta < minimum_turn - 1e-9)
        delta += 2 * pi;
      if (delta < best_delta) {
        best_delta = delta;
        best = heading;
      }
    }
    return best;
  }

private:
  std::vector<bool> observed_;
  int count_ = 0;
};
} // namespace powerplant
