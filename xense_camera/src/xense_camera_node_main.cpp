#include <rclcpp/rclcpp.hpp>
#include "xense_camera/xense_camera_node.hpp"

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<xense_camera::XenseCameraNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
