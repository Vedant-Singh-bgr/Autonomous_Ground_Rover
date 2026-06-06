
#ifndef CUSTOM_ROVER_NODES__DEPTH_DECODER_NODE_HPP_
#define CUSTOM_ROVER_NODES__DEPTH_DECODER_NODE_HPP_

#include "image_transport/image_transport.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "isaac_ros_tensor_list_interfaces/msg/tensor_list.hpp"
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>

namespace custom_rover_nodes
{

class DepthDecoderNode : public rclcpp::Node
{
public:
  explicit DepthDecoderNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

private:
  void tensor_callback(const isaac_ros_tensor_list_interfaces::msg::TensorList::SharedPtr msg);

  rclcpp::Subscription<isaac_ros_tensor_list_interfaces::msg::TensorList>::SharedPtr subscription_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr publisher_math_;
  image_transport::Publisher publisher_;
};

}  // namespace custom_rover_nodes

#endif  // CUSTOM_ROVER_NODES__DEPTH_DECODER_NODE_HPP_
