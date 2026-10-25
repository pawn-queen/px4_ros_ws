#include "px4_powerplant_mission_cpp/offboard.hpp"
#include "px4_powerplant_mission_cpp/input_validation.hpp"
#include <filesystem>
#include <iomanip>
#include <opencv2/imgcodecs.hpp>
namespace powerplant {
bool OffboardMissionNode::capture_photo(size_t waypoint, int goal) {
  if (photo_pending_)
    return false;
  auto fields = object();
  fields["waypoint_index"] = integer(waypoint);
  fields["photo_index"] = photos_ + 1;
  fields["goal_id"] = goal;
  if (target_) {
    fields["target_enu"] = vector_json(target_->position);
    fields["target_class"] = target_->class_name;
  }
  if (position_)
    fields["vehicle_enu"] = vector_json(*position_);
  if (!rgb_ ||
      !stamp_reason(rgb_sample_, ns(), d("max_photo_age_s"), d("max_future_stamp_skew_s")).empty() ||
      (steady_ns() - rgb_steady_) * 1e-9 > d("max_photo_age_s")) {
    fields["saved"] = false;
    fields["reason"] = "no_fresh_rgb_image";
    fields["image_required"] = b("require_image_for_photo");
    event("inspection_photo", fields);
    return false;
  }
  auto image = rgb_;
  std::string directory = s("inspection_photo_dir");
  int count = photos_ + 1;
  int64_t timestamp = ns() / 1000;
  photo_pending_ = true;
  photo_goal_ = goal;
  photo_final_ = waypoint + 1 >= waypoints_.size();
  recorder_.submit([image, directory, count, timestamp, fields](const auto &cancelled) mutable {
    PhotoResult result;
    result.event = fields;
    try {
      if (cancelled->load())
        return result;
      auto converted = cv_bridge::toCvCopy(*image, "bgr8");
      std::string path = directory;
      if (path == "~" || path.rfind("~/", 0) == 0) {
        const char *home = std::getenv("HOME");
        if (home)
          path = std::string(home) + path.substr(1);
      }
      std::filesystem::create_directories(path);
      std::ostringstream name;
      name << "inspection_" << std::setfill('0') << std::setw(2) << count << "_"
           << timestamp << ".jpg";
      auto filename = std::filesystem::path(path) / name.str();
      if (cancelled->load())
        return result;
      result.success = cv::imwrite(filename.string(), converted->image);
      result.event["saved"] = result.success;
      result.event["path"] = filename.string();
    } catch (const std::exception &e) {
      result.event["saved"] = false;
      result.event["reason"] = e.what();
      result.success = false;
    }
    return result;
  });
  return false;
}
} // namespace powerplant
