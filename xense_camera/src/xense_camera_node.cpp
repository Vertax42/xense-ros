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
  publish_fps_ = get_parameter("publish_fps").as_double();
  qos_reliable_ = get_parameter("qos_reliable").as_bool();

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

  // Start publish thread before pipeline so it is ready when frames arrive.
  publish_thread_running_ = true;
  publish_thread_ = std::thread(&XenseCameraNode::publish_loop, this);

  // WallTimer runs at publish_fps_ Hz on the executor thread.
  // Its only job is notify_one() — never blocks the executor.
  const auto period = std::chrono::duration_cast<std::chrono::nanoseconds>(
    std::chrono::duration<double>(1.0 / publish_fps_));
  publish_timer_ = create_wall_timer(period,
    std::bind(&XenseCameraNode::publish_timer_cb, this));

  auto config = build_pipeline_config();

  RCLCPP_INFO(get_logger(), "Starting xense pipeline...");
  RCLCPP_INFO(get_logger(), "  device_serial      : '%s'", device_serial_.c_str());
  RCLCPP_INFO(get_logger(), "  enable_raw         : %s", enable_raw_ ? "true" : "false");
  RCLCPP_INFO(get_logger(), "  enable_rectified   : %s", enable_rectified_ ? "true" : "false");
  RCLCPP_INFO(get_logger(), "  enable_diff        : %s", enable_diff_ ? "true" : "false");
  RCLCPP_INFO(get_logger(), "  diff_mode          : %s", diff_mode_.c_str());
  RCLCPP_INFO(get_logger(), "  inference_backend  : %s", inference_backend_.c_str());
  RCLCPP_INFO(get_logger(), "  publish_fps        : %.1f", publish_fps_);

  pipeline_.start(config, [this](xense::FrameSet frames) {
    on_frame_set(std::move(frames));
  });

  RCLCPP_INFO(get_logger(), "Xense camera node started.");
}

XenseCameraNode::~XenseCameraNode()
{
  publish_timer_->cancel();

  publish_thread_running_ = false;
  publish_cv_.notify_all();
  if (publish_thread_.joinable()) {
    publish_thread_.join();
  }

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
  declare_parameter<double>("publish_fps", 30.0);
  declare_parameter<bool>("qos_reliable", true);
}

xense::PipelineConfig XenseCameraNode::build_pipeline_config()
{
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
  const rmw_qos_profile_t img_qos =
    qos_reliable_ ? rmw_qos_profile_default : rmw_qos_profile_sensor_data;
  const rclcpp::QoS info_qos =
    qos_reliable_ ? rclcpp::QoS(10) : rclcpp::SensorDataQoS();

  camera_info_pub_ = create_publisher<sensor_msgs::msg::CameraInfo>(
    "~/camera_info", info_qos);

  if (enable_raw_) {
    raw_pub_ = image_transport::create_publisher(
      this, "~/raw/image_raw", img_qos);
  }
  if (enable_rectified_ || enable_diff_) {
    rectified_pub_ = image_transport::create_publisher(
      this, "~/rectified/image", img_qos);
  }
  if (enable_diff_) {
    diff_pub_ = image_transport::create_publisher(
      this, "~/diff/image", img_qos);
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

// Called from SDK capture thread — must return immediately.
// notify_one() ensures the publish thread wakes immediately when a frame
// arrives, even if the WallTimer has not fired yet.
void XenseCameraNode::on_frame_set(xense::FrameSet frames)
{
  if (frames.empty()) {
    return;
  }

  // Log SDK frame delivery interval using hardware timestamps.
  // If gaps appear here, the delay is in the SDK/hardware layer, not ROS.
  if (auto first = frames.first(); first.valid()) {
    static int64_t last_ts_us = 0;
    const int64_t ts_us = first.timestamp_us();
    if (last_ts_us != 0) {
      const double interval_ms = (ts_us - last_ts_us) / 1000.0;
      if (interval_ms > 50.0) {
        RCLCPP_WARN(get_logger(),
          "[on_frame_set] Large SDK frame gap: %.1f ms (hardware ts)", interval_ms);
      } else {
        RCLCPP_DEBUG(get_logger(),
          "[on_frame_set] SDK frame interval: %.1f ms", interval_ms);
      }
    }
    last_ts_us = ts_us;
  }

  {
    std::lock_guard<std::mutex> lock(frame_mutex_);
    latest_frame_ = std::move(frames);
    has_new_frame_ = true;
  }
  publish_cv_.notify_one();
}

// Called by WallTimer on the executor thread — O(1), never blocks.
// Acts as a rate-limiter: if a frame arrived before the tick, this is a no-op
// (thread already published). If a frame arrives after the tick, the
// on_frame_set notify will wake the thread without waiting for the next tick.
void XenseCameraNode::publish_timer_cb()
{
  publish_cv_.notify_one();
}

// Publish thread: wakes on each timer tick, grabs the latest frame, publishes.
// Decouples heavy frame conversion from the executor thread so the WallTimer
// always fires on time.
void XenseCameraNode::publish_loop()
{
  while (publish_thread_running_) {
    {
      std::unique_lock<std::mutex> lock(frame_mutex_);
      publish_cv_.wait(lock, [this] {
        return has_new_frame_ || !publish_thread_running_;
      });

      if (!publish_thread_running_) {
        break;
      }
    }

    xense::FrameSet frames;
    {
      std::lock_guard<std::mutex> lock(frame_mutex_);
      if (!has_new_frame_) {
        continue;
      }
      frames = std::move(latest_frame_);
      has_new_frame_ = false;
    }

    publish_frame_set(frames);
  }
}

void XenseCameraNode::publish_frame_set(xense::FrameSet & frames)
{
  using clock = std::chrono::steady_clock;
  using ms = std::chrono::duration<double, std::milli>;

  static clock::time_point last_publish_wall;
  const auto publish_wall = clock::now();

  if (last_publish_wall.time_since_epoch().count() != 0) {
    const double interval_ms = ms(publish_wall - last_publish_wall).count();
    if (interval_ms > 50.0) {
      RCLCPP_WARN(get_logger(), "[publish] Large publish gap: %.1f ms", interval_ms);
    } else {
      RCLCPP_DEBUG(get_logger(), "[publish] Publish interval: %.1f ms", interval_ms);
    }
  }

  // Internal FPS counter: measures node publish rate regardless of DDS subscriber load.
  // Use this (or `ros2 topic hz ~/camera_info`) instead of `ros2 topic hz ~/rectified/image`
  // — large image messages can overflow the Python subscriber's BEST_EFFORT queue (depth=5).
  {
    static uint32_t frame_count = 0;
    static clock::time_point fps_window_start = publish_wall;
    ++frame_count;
    const double window_ms = ms(publish_wall - fps_window_start).count();
    if (window_ms >= 3000.0) {
      RCLCPP_INFO(get_logger(), "[fps] Node publish rate: %.2f Hz (%u frames / %.1f s)",
        frame_count * 1000.0 / window_ms, frame_count, window_ms / 1000.0);
      frame_count = 0;
      fps_window_start = publish_wall;
    }
  }

  // Latency: SDK hardware timestamp vs wall clock (both steady_clock-based)
  if (auto first = frames.first(); first.valid()) {
    const int64_t sdk_us = first.timestamp_us();
    const int64_t now_us = std::chrono::duration_cast<std::chrono::microseconds>(
      publish_wall.time_since_epoch()).count();
    RCLCPP_DEBUG(get_logger(), "[publish] Frame-to-publish latency: %.1f ms",
      (now_us - sdk_us) / 1000.0);
  }

  rclcpp::Time stamp = now();
  if (auto first = frames.first(); first.valid()) {
    stamp = rclcpp::Time(first.timestamp_us() * 1000LL);
  }

  auto cam_info = make_camera_info(
    frames.first().width(),
    frames.first().height(),
    camera_frame_id_,
    stamp);
  camera_info_pub_->publish(cam_info);

  if (enable_raw_ && frames.contains(xense::StreamType::Raw)) {
    auto frame = frames.get(xense::StreamType::Raw);
    if (frame.valid()) {
      try {
        raw_pub_.publish(frame_to_bgr8(frame, camera_frame_id_));
      } catch (const std::exception & e) {
        RCLCPP_WARN(get_logger(), "Failed to publish raw frame: %s", e.what());
      }
    }
  }

  if ((enable_rectified_ || enable_diff_) &&
    frames.contains(xense::StreamType::Rectified))
  {
    auto frame = frames.get(xense::StreamType::Rectified);
    if (frame.valid()) {
      try {
        rectified_pub_.publish(frame_to_bgr8(frame, camera_frame_id_));
      } catch (const std::exception & e) {
        RCLCPP_WARN(get_logger(), "Failed to publish rectified frame: %s", e.what());
      }
    }
  }

  if (enable_diff_ && frames.contains(xense::StreamType::Diff)) {
    auto frame = frames.get(xense::StreamType::Diff);
    if (frame.valid()) {
      try {
        sensor_msgs::msg::Image msg = (frame.format() == xense::FrameFormat::Float32)
          ? frame_to_float32(frame, camera_frame_id_)
          : frame_to_bgr8(frame, camera_frame_id_);
        diff_pub_.publish(msg);
      } catch (const std::exception & e) {
        RCLCPP_WARN(get_logger(), "Failed to publish diff frame: %s", e.what());
      }
    }
  }

  last_publish_wall = publish_wall;
}

}  // namespace xense_camera
