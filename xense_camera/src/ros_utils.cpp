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

  sensor_msgs::msg::Image msg;
  msg.header.stamp = rclcpp::Time(frame.timestamp_us() * 1000LL);  // us -> ns
  msg.header.frame_id = frame_id;
  msg.width = static_cast<uint32_t>(frame.width());
  msg.height = static_cast<uint32_t>(frame.height());
  msg.encoding = "bgr8";
  msg.step = static_cast<uint32_t>(frame.stride());
  msg.is_bigendian = false;

  const size_t data_size = frame.data_size();
  msg.data.resize(data_size);
  std::memcpy(msg.data.data(), frame.data(), data_size);

  return msg;
}

sensor_msgs::msg::Image frame_to_float32(
  const xense::Frame & frame,
  const std::string & frame_id)
{
  if (!frame.valid()) {
    throw std::runtime_error("frame_to_float32: invalid frame");
  }

  sensor_msgs::msg::Image msg;
  msg.header.stamp = rclcpp::Time(frame.timestamp_us() * 1000LL);  // us -> ns
  msg.header.frame_id = frame_id;
  msg.width = static_cast<uint32_t>(frame.width());
  msg.height = static_cast<uint32_t>(frame.height());
  msg.encoding = "32FC1";
  msg.step = static_cast<uint32_t>(frame.stride());
  msg.is_bigendian = false;

  const size_t data_size = frame.data_size();
  msg.data.resize(data_size);
  std::memcpy(msg.data.data(), frame.data(), data_size);

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
