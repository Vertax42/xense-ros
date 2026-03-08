#pragma once

#include <memory>
#include <string>
#include <mutex>
#include <condition_variable>
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
  bool enable_diff_single_;
  bool enable_diff_continuous_;
  std::string inference_backend_;
  bool use_gpu_;
  std::string camera_frame_id_;
  bool publish_tf_;
  double publish_fps_;

  // SDK objects
  xense::Pipeline pipeline_;

  // Publishers
  image_transport::Publisher raw_pub_;
  image_transport::Publisher rectified_pub_;
  image_transport::Publisher diff_single_pub_;
  image_transport::Publisher diff_continuous_pub_;
  rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr camera_info_pub_;

  // TF
  std::shared_ptr<tf2_ros::StaticTransformBroadcaster> static_tf_broadcaster_;

  // Latest frame slot: SDK callback writes here
  xense::FrameSet latest_frame_;
  std::mutex frame_mutex_;
  bool has_new_frame_{false};

  // WallTimer signals the publish thread; heavy work stays off the executor
  rclcpp::TimerBase::SharedPtr publish_timer_;
  std::condition_variable publish_cv_;
  std::thread publish_thread_;
  std::atomic<bool> publish_thread_running_{false};

  void declare_parameters();
  xense::PipelineConfig build_pipeline_config();
  void create_publishers();
  void publish_static_tf();
  void on_frame_set(xense::FrameSet frames);  // SDK thread: stores latest frame
  void publish_timer_cb();                     // executor thread: signals publish thread (O(1))
  void publish_loop();                         // publish thread: does actual frame conversion
  void publish_frame_set(xense::FrameSet & frames);
};

}  // namespace xense_camera
