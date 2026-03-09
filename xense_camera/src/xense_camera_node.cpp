#include "xense_camera/xense_camera_node.hpp"
#include "xense_camera/ros_utils.hpp"

#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2_ros/static_transform_broadcaster.h>

namespace xense_camera
{

XenseCameraNode::XenseCameraNode(const rclcpp::NodeOptions & options)
: Node("xense_camera", options)
{
  declare_parameters();

  // Retrieve parameters
  device_serial_ = get_parameter("device_serial").as_string();
  device_index_ = get_parameter("device_index").as_int();
  enable_raw_ = get_parameter("enable_raw").as_bool();
  enable_rectified_ = get_parameter("enable_rectified").as_bool();
  enable_diff_ = get_parameter("enable_diff").as_bool();
  diff_mode_ = get_parameter("diff_mode").as_string();
  inference_backend_ = get_parameter("inference_backend").as_string();
  use_gpu_ = get_parameter("use_gpu").as_bool();
  camera_frame_id_ = get_parameter("camera_frame_id").as_string();
  publish_tf_ = get_parameter("publish_tf").as_bool();

  // Require at least one stream
  if (!enable_raw_ && !enable_rectified_ && !enable_diff_) {
    RCLCPP_WARN(get_logger(),
      "No streams enabled. Defaulting to enable_rectified=true.");
    enable_rectified_ = true;
  }

  create_publishers();

  if (publish_tf_) {
    static_tf_broadcaster_ =
      std::make_shared<tf2_ros::StaticTransformBroadcaster>(this);
    publish_static_tf();
  }

  // Build and start pipeline
  auto config = build_pipeline_config();

  RCLCPP_INFO(get_logger(), "Starting xense pipeline...");
  RCLCPP_INFO(get_logger(), "  device_serial : '%s'", device_serial_.c_str());
  RCLCPP_INFO(get_logger(), "  enable_raw    : %s", enable_raw_ ? "true" : "false");
  RCLCPP_INFO(get_logger(), "  enable_rectified: %s", enable_rectified_ ? "true" : "false");
  RCLCPP_INFO(get_logger(), "  enable_diff   : %s", enable_diff_ ? "true" : "false");
  RCLCPP_INFO(get_logger(), "  diff_mode     : %s", diff_mode_.c_str());
  RCLCPP_INFO(get_logger(), "  inference_backend: %s", inference_backend_.c_str());

  pipeline_.start(config, [this](xense::FrameSet frames) {
    on_frame_set(std::move(frames));
  });

  RCLCPP_INFO(get_logger(), "Xense camera node started.");
}

XenseCameraNode::~XenseCameraNode()
{
  try {
    if (pipeline_.is_running()) {
      pipeline_.stop();
    }
  } catch (const std::exception & e) {
    RCLCPP_ERROR(get_logger(), "Error stopping pipeline: %s", e.what());
  }
}

void XenseCameraNode::declare_parameters()
{
  declare_parameter<std::string>("device_serial", "");
  declare_parameter<int>("device_index", -1);
  declare_parameter<bool>("enable_raw", false);
  declare_parameter<bool>("enable_rectified", true);
  declare_parameter<bool>("enable_diff", false);
  declare_parameter<std::string>("diff_mode", "single");
  declare_parameter<std::string>("inference_backend", "Auto");
  declare_parameter<bool>("use_gpu", true);
  declare_parameter<std::string>("camera_frame_id", "xense_camera_link");
  declare_parameter<bool>("publish_tf", true);
}

xense::PipelineConfig XenseCameraNode::build_pipeline_config()
{
  // Collect the set of required streams, respecting upstream dependencies.
  // Diff requires Rectified; Raw is independent.
  std::vector<std::string> streams;

  if (enable_raw_) {
    streams.push_back("Raw");
  }
  if (enable_rectified_ || enable_diff_) {
    streams.push_back("Rectified");
  }
  if (enable_diff_) {
    streams.push_back("Diff");
  }

  xense::PipelineConfig config;
  config.streams = streams;
  config.diff_mode = (diff_mode_ == "continuous") ? "PerFrameInference" : "SingleInference";
  config.inference_backend = inference_backend_;
  config.use_gpu = use_gpu_;

  if (!device_serial_.empty()) {
    config.device_serial = device_serial_;
  }
  if (device_index_ >= 0) {
    config.device_index = device_index_;
  }

  return config;
}

void XenseCameraNode::create_publishers()
{
  camera_info_pub_ = create_publisher<sensor_msgs::msg::CameraInfo>(
    "~/camera_info", rclcpp::QoS(10));

  if (enable_raw_) {
    raw_pub_ = image_transport::create_publisher(this, "~/raw/image_raw");
  }
  if (enable_rectified_ || enable_diff_) {
    rectified_pub_ = image_transport::create_publisher(this, "~/rectified/image");
  }
  if (enable_diff_) {
    diff_pub_ = image_transport::create_publisher(this, "~/diff/image");
  }
}

void XenseCameraNode::publish_static_tf()
{
  geometry_msgs::msg::TransformStamped tf_msg;
  tf_msg.header.stamp = now();
  tf_msg.header.frame_id = "world";
  tf_msg.child_frame_id = camera_frame_id_;
  tf_msg.transform.translation.x = 0.0;
  tf_msg.transform.translation.y = 0.0;
  tf_msg.transform.translation.z = 0.0;
  tf_msg.transform.rotation.x = 0.0;
  tf_msg.transform.rotation.y = 0.0;
  tf_msg.transform.rotation.z = 0.0;
  tf_msg.transform.rotation.w = 1.0;
  static_tf_broadcaster_->sendTransform(tf_msg);
}

void XenseCameraNode::on_frame_set(xense::FrameSet frames)
{
  if (frames.empty()) {
    return;
  }

  // Determine timestamp from the first available frame
  rclcpp::Time stamp = now();
  if (auto first = frames.first(); first.valid()) {
    stamp = rclcpp::Time(first.timestamp_us() * 1000LL);
  }

  // Publish camera_info once per frame set
  auto cam_info = make_camera_info(
    frames.first().width(),
    frames.first().height(),
    camera_frame_id_,
    stamp);
  camera_info_pub_->publish(cam_info);

  // Raw
  if (enable_raw_ && frames.contains(xense::StreamType::Raw)) {
    auto frame = frames.get(xense::StreamType::Raw);
    if (frame.valid()) {
      try {
        auto msg = frame_to_bgr8(frame, camera_frame_id_);
        raw_pub_.publish(msg);
      } catch (const std::exception & e) {
        RCLCPP_WARN(get_logger(), "Failed to publish raw frame: %s", e.what());
      }
    }
  }

  // Rectified
  if ((enable_rectified_ || enable_diff_) &&
    frames.contains(xense::StreamType::Rectified))
  {
    auto frame = frames.get(xense::StreamType::Rectified);
    if (frame.valid()) {
      try {
        auto msg = frame_to_bgr8(frame, camera_frame_id_);
        rectified_pub_.publish(msg);
      } catch (const std::exception & e) {
        RCLCPP_WARN(get_logger(), "Failed to publish rectified frame: %s", e.what());
      }
    }
  }

  // Diff
  if (enable_diff_ && frames.contains(xense::StreamType::Diff)) {
    auto frame = frames.get(xense::StreamType::Diff);
    if (frame.valid()) {
      try {
        sensor_msgs::msg::Image msg;
        if (frame.format() == xense::FrameFormat::Float32) {
          msg = frame_to_float32(frame, camera_frame_id_);
        } else {
          msg = frame_to_bgr8(frame, camera_frame_id_);
        }
        diff_pub_.publish(msg);
      } catch (const std::exception & e) {
        RCLCPP_WARN(get_logger(), "Failed to publish diff frame: %s", e.what());
      }
    }
  }
}

}  // namespace xense_camera
