#pragma once

#include <memory>
#include <string>
#include <mutex>
#include <thread>
#include <atomic>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <image_transport/image_transport.hpp>
#include <tf2_ros/static_transform_broadcaster.h>

#include <xense/xense.hpp>

namespace xense_camera
{

class XenseCameraNode : public rclcpp::Node
{
public:
  explicit XenseCameraNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());
  ~XenseCameraNode();

private:
  // Parameters
  std::string device_serial_;
  int device_index_;
  bool enable_raw_;
  bool enable_rectified_;
  bool enable_diff_;
  bool enable_depth_;
  std::string diff_mode_;
  std::string inference_backend_;
  bool use_gpu_;
  std::string camera_frame_id_;
  bool publish_tf_;

  // SDK objects
  xense::Pipeline pipeline_;

  // Publishers
  image_transport::Publisher raw_pub_;
  image_transport::Publisher rectified_pub_;
  image_transport::Publisher diff_pub_;
  image_transport::Publisher depth_pub_;
  rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr camera_info_pub_;

  // TF
  std::shared_ptr<tf2_ros::StaticTransformBroadcaster> static_tf_broadcaster_;

  // Latest frame slot: SDK callback writes, publish thread reads at fixed rate
  xense::FrameSet latest_frame_;
  std::mutex frame_mutex_;
  bool has_new_frame_{false};
  std::thread publish_thread_;
  std::atomic<bool> publish_thread_running_{false};

  // Internal
  void declare_parameters();
  xense::PipelineConfig build_pipeline_config();
  void create_publishers();
  void publish_static_tf();
  void on_frame_set(xense::FrameSet frames);  // SDK thread: stores latest frame
  void publish_loop();                         // dedicated thread: fixed-rate sleep_until
  void publish_frame_set(xense::FrameSet & frames);

  double publish_fps_;
};

}  // namespace xense_camera
