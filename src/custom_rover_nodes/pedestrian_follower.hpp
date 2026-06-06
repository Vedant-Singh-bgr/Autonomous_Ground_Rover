#ifndef CUSTOM_ROVER_NODES__PEDESTRIAN_FOLLOWER_HPP_
#define CUSTOM_ROVER_NODES__PEDESTRIAN_FOLLOWER_HPP_

#include "image_transport/image_transport.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "isaac_ros_tensor_list_interfaces/msg/tensor_list.hpp"
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include "vision_msgs/msg/detection2_d_array.hpp"
#include <std_msgs/msg/float32.hpp>

#include "message_filters/subscriber.h"
#include "message_filters/sync_policies/approximate_time.h"
#include "message_filters/synchronizer.h"

namespace custom_rover_nodes
{

class PedestrianFollower: public rclcpp::Node
{
public:
  explicit PedestrianFollower(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

private:
  void synced_callback(
    const vision_msgs::msg::Detection2DArray::ConstSharedPtr& detection_msg,
    const isaac_ros_tensor_list_interfaces::msg::TensorList::ConstSharedPtr& depth_msg);
  message_filters::Subscriber<vision_msgs::msg::Detection2DArray> detection_sub_;
  message_filters::Subscriber<isaac_ros_tensor_list_interfaces::msg::TensorList> depth_sub_;
  typedef message_filters::sync_policies::ApproximateTime<
    vision_msgs::msg::Detection2DArray,
    isaac_ros_tensor_list_interfaces::msg::TensorList> SyncPolicy;
  std::shared_ptr<message_filters::Synchronizer<SyncPolicy>> sync_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr publisher_math_;
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr distance_pub_;
  
};

}  // namespace custom_rover_nodes

#endif  // CUSTOM_ROVER_NODES__DEPTH_DECODER_NODE_HPP_