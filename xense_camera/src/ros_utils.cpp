#include "xense_camera/ros_utils.hpp"

#include <cstring>
#include <stdexcept>

namespace xense_camera
{

sensor_msgs::msg::Image frame_to_bgr8(
  const xense::Frame & frame,
  const std::string & frame_id)
{
  if (!frame.valid()) {
    throw std::runtime_error("frame_to_bgr8: invalid frame");
  }

  const uint32_t w = static_cast<uint32_t>(frame.width());
  const uint32_t h = static_cast<uint32_t>(frame.height());
  const uint32_t step = w * 3u;  // bgr8: 3 bytes per pixel, no padding

  sensor_msgs::msg::Image msg;
  msg.header.stamp = rclcpp::Time(frame.timestamp_us() * 1000LL);
  msg.header.frame_id = frame_id;
  msg.width = w;
  msg.height = h;
  msg.encoding = "bgr8";
  msg.step = step;
  msg.is_bigendian = false;

  const size_t expected = static_cast<size_t>(h) * step;
  msg.data.resize(expected);
  std::memcpy(msg.data.data(), frame.data(), std::min(expected, frame.data_size()));

  return msg;
}

sensor_msgs::msg::Image frame_to_float32(
  const xense::Frame & frame,
  const std::string & frame_id)
{
  if (!frame.valid()) {
    throw std::runtime_error("frame_to_float32: invalid frame");
  }

  const uint32_t w = static_cast<uint32_t>(frame.width());
  const uint32_t h = static_cast<uint32_t>(frame.height());
  const uint32_t step = w * 4u;  // 32FC1: 4 bytes per pixel, no padding

  sensor_msgs::msg::Image msg;
  msg.header.stamp = rclcpp::Time(frame.timestamp_us() * 1000LL);
  msg.header.frame_id = frame_id;
  msg.width = w;
  msg.height = h;
  msg.encoding = "32FC1";
  msg.step = step;
  msg.is_bigendian = false;

  const size_t expected = static_cast<size_t>(h) * step;
  msg.data.resize(expected);
  std::memcpy(msg.data.data(), frame.data(), std::min(expected, frame.data_size()));

  return msg;
}

sensor_msgs::msg::CameraInfo make_camera_info(
  int width,
  int height,
  const std::string & frame_id,
  const rclcpp::Time & stamp)
{
  sensor_msgs::msg::CameraInfo msg;
  msg.header.stamp = stamp;
  msg.header.frame_id = frame_id;
  msg.width = static_cast<uint32_t>(width);
  msg.height = static_cast<uint32_t>(height);
  msg.distortion_model = "plumb_bob";
  // Calibration parameters are left as zeros (uncalibrated).
  // Users should supply a camera_info_manager or yaml calibration file.
  return msg;
}

}  // namespace xense_camera
