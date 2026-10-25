#pragma once
#include "px4_powerplant_mission_cpp/core.hpp"
#include <cstring>
#include <cv_bridge/cv_bridge.hpp>
#include <json/json.h>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sstream>
#include <std_msgs/msg/string.hpp>

namespace powerplant {
using JsonValue = Json::Value;
inline JsonValue object() { return JsonValue(Json::objectValue); }
inline JsonValue array() { return JsonValue(Json::arrayValue); }
inline JsonValue integer(int64_t n) { return JsonValue(Json::Int64(n)); }
inline JsonValue number(double v, int digits = 3) {
  if (!std::isfinite(v))
    return {};
  double scale = std::pow(10, digits);
  return std::nearbyint(v * scale) / scale;
}
inline JsonValue vector_json(const Vec3 &p, int digits = 2) {
  auto a = array();
  for (int i = 0; i < 3; ++i)
    a.append(number(p[i], digits));
  return a;
}
inline JsonValue key_json(const Key &k) {
  auto a = array();
  for (int v : k)
    a.append(v);
  return a;
}
inline std::string serialize(const JsonValue &v) {
  Json::StreamWriterBuilder writer;
  writer["indentation"] = "";
  return Json::writeString(writer, v);
}
inline JsonValue parse(const std::string &text) {
  Json::CharReaderBuilder b;
  b["failIfExtra"] = true;
  JsonValue result;
  std::string error;
  std::istringstream stream(text);
  if (!Json::parseFromStream(b, stream, &result, &error) || !result.isObject())
    throw std::invalid_argument("invalid JSON object");
  return result;
}
inline void
publish_json(const rclcpp::Publisher<std_msgs::msg::String>::SharedPtr &pub,
             const JsonValue &v) {
  std_msgs::msg::String msg;
  msg.data = serialize(v);
  pub->publish(msg);
}
inline rclcpp::QoS px4_qos() {
  return rclcpp::QoS(1).best_effort().transient_local();
}
inline rclcpp::QoS sensor_qos() {
  return rclcpp::QoS(5).best_effort().durability_volatile();
}
inline rclcpp::QoS reliable_qos() {
  return rclcpp::QoS(10).reliable().durability_volatile();
}
inline int64_t stamp_ns(const builtin_interfaces::msg::Time &stamp) {
  return int64_t(stamp.sec) * 1000000000 + stamp.nanosec;
}
// Detect mismatched payloads when a clock reset reuses an old cloud stamp.
// This is an integrity checksum, not an authentication mechanism. Include the
// wire layout as well as bytes so differently interpreted clouds cannot pair.
inline std::string cloud_fingerprint(const sensor_msgs::msg::PointCloud2 &cloud) {
  uint64_t hash = 14695981039346656037ULL;
  auto byte = [&](uint8_t value) {
    hash ^= value;
    hash *= 1099511628211ULL;
  };
  auto integer_bytes = [&](uint64_t value) {
    for (int shift = 0; shift < 64; shift += 8)
      byte(static_cast<uint8_t>(value >> shift));
  };
  auto string_bytes = [&](const std::string &value) {
    integer_bytes(value.size());
    for (unsigned char character : value)
      byte(character);
  };
  string_bytes(cloud.header.frame_id);
  integer_bytes(cloud.width);
  integer_bytes(cloud.height);
  integer_bytes(cloud.point_step);
  integer_bytes(cloud.row_step);
  integer_bytes(cloud.is_bigendian);
  integer_bytes(cloud.fields.size());
  for (const auto &field : cloud.fields) {
    string_bytes(field.name);
    integer_bytes(field.offset);
    integer_bytes(field.datatype);
    integer_bytes(field.count);
  }
  integer_bytes(cloud.data.size());
  for (uint8_t value : cloud.data)
    byte(value);
  std::ostringstream result;
  result << std::hex << hash;
  return result.str();
}
class ParameterNode : public rclcpp::Node {
public:
  explicit ParameterNode(const std::string &name) : Node(name) {}
  double d(const std::string &n) const { return get_parameter(n).as_double(); }
  int i(const std::string &n) const { return int(get_parameter(n).as_int()); }
  bool b(const std::string &n) const { return get_parameter(n).as_bool(); }
  std::string s(const std::string &n) const {
    return get_parameter(n).as_string();
  }
  Vec3 v(const std::string &n) const {
    auto a = get_parameter(n).as_double_array();
    if (a.size() != 3)
      throw std::invalid_argument(n + " must contain three values");
    return {a[0], a[1], a[2]};
  }
  int64_t ns() const { return get_clock()->now().nanoseconds(); }
  double age(int64_t t) const { return t <= 0 ? inf : (ns() - t) * 1e-9; }
};
inline cv::Mat depth_image(const sensor_msgs::msg::Image &msg) {
  auto cv = cv_bridge::toCvCopy(msg);
  cv::Mat channel;
  if (cv->image.channels() > 1)
    cv::extractChannel(cv->image, channel, 0);
  else
    channel = cv->image;
  cv::Mat depth;
  channel.convertTo(depth, CV_32F,
                    (msg.encoding == "16UC1" || msg.encoding == "mono16") ? .001
                                                                          : 1);
  return depth;
}
inline double median(std::vector<float> &values) {
  if (values.empty())
    return std::numeric_limits<double>::quiet_NaN();
  auto middle = values.begin() + values.size() / 2;
  std::nth_element(values.begin(), middle, values.end());
  double high = *middle;
  if (values.size() % 2)
    return high;
  return .5 * (high + *std::max_element(values.begin(), middle));
}
inline sensor_msgs::msg::PointCloud2
cloud_message(const std_msgs::msg::Header &header,
              const std::vector<std::array<float, 4>> &points,
              bool state = true) {
  sensor_msgs::msg::PointCloud2 msg;
  msg.header = header;
  msg.height = 1;
  msg.width = points.size();
  msg.is_dense = true;
  const std::array<std::string, 4> names = {"x", "y", "z", "state"};
  int count = state ? 4 : 3;
  for (int i = 0; i < count; ++i) {
    sensor_msgs::msg::PointField f;
    f.name = names[i];
    f.offset = i * 4;
    f.count = 1;
    f.datatype = f.FLOAT32;
    msg.fields.push_back(f);
  }
  msg.point_step = count * 4;
  msg.row_step = msg.width * msg.point_step;
  msg.data.resize(msg.row_step);
  for (size_t i = 0; i < points.size(); ++i)
    std::memcpy(msg.data.data() + i * msg.point_step, points[i].data(),
                msg.point_step);
  return msg;
}
// Respect offsets, row padding, field types and wire endianness, rather than
// casting the payload.
template <class T> inline double wire_scalar(const uint8_t *data, bool swap) {
  std::array<uint8_t, sizeof(T)> bytes;
  std::memcpy(bytes.data(), data, sizeof(T));
  if (swap)
    std::reverse(bytes.begin(), bytes.end());
  T value;
  std::memcpy(&value, bytes.data(), sizeof(T));
  return static_cast<double>(value);
}
inline std::vector<std::array<double, 4>>
read_cloud(const sensor_msgs::msg::PointCloud2 &msg, bool state) {
  using Field = sensor_msgs::msg::PointField;
  const int count = state ? 4 : 3;
  const std::array<std::string, 4> names = {"x", "y", "z", "state"};
  std::array<uint32_t, 4> offsets{};
  std::array<uint8_t, 4> types{};
  if (msg.point_step == 0 ||
      uint64_t(msg.width) * msg.point_step > msg.row_step ||
      uint64_t(msg.row_step) * msg.height > msg.data.size())
    throw std::invalid_argument("invalid point cloud layout");
  for (int j = 0; j < count; ++j) {
    auto it = std::find_if(msg.fields.begin(), msg.fields.end(),
                           [&](const auto &f) { return f.name == names[j]; });
    if (it == msg.fields.end() || it->count != 1 ||
        (state && it->datatype != Field::FLOAT32))
      throw std::invalid_argument("missing or invalid point cloud field");
    uint32_t size = 0;
    switch (it->datatype) {
    case Field::INT8:
    case Field::UINT8:
      size = 1;
      break;
    case Field::INT16:
    case Field::UINT16:
      size = 2;
      break;
    case Field::INT32:
    case Field::UINT32:
    case Field::FLOAT32:
      size = 4;
      break;
    case Field::FLOAT64:
      size = 8;
      break;
    default:
      throw std::invalid_argument("unsupported point cloud field");
    }
    if (uint64_t(it->offset) + size > msg.point_step)
      throw std::invalid_argument("point field exceeds stride");
    offsets[j] = it->offset;
    types[j] = it->datatype;
  }
  uint16_t endian = 1;
  const bool swap =
      (*reinterpret_cast<uint8_t *>(&endian) == 0) != msg.is_bigendian;
  std::vector<std::array<double, 4>> out;
  out.reserve(size_t(msg.width) * msg.height);
  for (uint32_t y = 0; y < msg.height; ++y)
    for (uint32_t x = 0; x < msg.width; ++x) {
      std::array<double, 4> p{};
      for (int j = 0; j < count; ++j) {
        const auto *data = msg.data.data() + size_t(y) * msg.row_step +
                           size_t(x) * msg.point_step + offsets[j];
        switch (types[j]) {
        case Field::INT8:
          p[j] = wire_scalar<int8_t>(data, swap);
          break;
        case Field::UINT8:
          p[j] = wire_scalar<uint8_t>(data, swap);
          break;
        case Field::INT16:
          p[j] = wire_scalar<int16_t>(data, swap);
          break;
        case Field::UINT16:
          p[j] = wire_scalar<uint16_t>(data, swap);
          break;
        case Field::INT32:
          p[j] = wire_scalar<int32_t>(data, swap);
          break;
        case Field::UINT32:
          p[j] = wire_scalar<uint32_t>(data, swap);
          break;
        case Field::FLOAT32:
          p[j] = wire_scalar<float>(data, swap);
          break;
        case Field::FLOAT64:
          p[j] = wire_scalar<double>(data, swap);
          break;
        }
      }
      out.push_back(p);
    }
  return out;
}

} // namespace powerplant
