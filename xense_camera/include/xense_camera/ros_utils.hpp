#pragma once

#include <string>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <xense/xense.hpp>

namespace xense_camera
{

/// Convert an xense::Frame (BGR8) to a sensor_msgs/Image with bgr8 encoding.
sensor_msgs::msg::Image frame_to_bgr8(
  const xense::Frame & frame,
  const std::string & frame_id);

/// Convert an xense::Frame (Float32) to a sensor_msgs/Image with 32FC1 encoding.
sensor_msgs::msg::Image frame_to_float32(
  const xense::Frame & frame,
  const std::string & frame_id);

/// Build a minimal CameraInfo message (no calibration) with the given image size.
sensor_msgs::msg::CameraInfo make_camera_info(
  int width,
  int height,
  const std::string & frame_id,
  const rclcpp::Time & stamp);

}  // namespace xense_camera
