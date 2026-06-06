#include "custom_rover_nodes/depth_decoder_node.hpp"
#include "rclcpp_components/register_node_macro.hpp"
#include <cv_bridge/cv_bridge.h>
namespace custom_rover_nodes
{

DepthDecoderNode::DepthDecoderNode(const rclcpp::NodeOptions & options)
: Node("depth_decoder_node", options)
{
  rclcpp::SubscriptionOptions sub_options;
  sub_options.use_intra_process_comm = rclcpp::IntraProcessSetting::Enable;

  subscription_ = this->create_subscription<isaac_ros_tensor_list_interfaces::msg::TensorList>(
    "/depth_tensor_output", 10,
    std::bind(&DepthDecoderNode::tensor_callback, this, std::placeholders::_1),sub_options);
  rclcpp::PublisherOptions pub_options;
  pub_options.use_intra_process_comm = rclcpp::IntraProcessSetting::Enable;
  publisher_math_ = this->create_publisher<sensor_msgs::msg::Image>("/camera/depth/image_raw_disparity", 10, pub_options);
  publisher_ = image_transport::create_publisher(this,"/camera/depth/image_decoded");
  
  
  RCLCPP_INFO(this->get_logger(), "C++ Depth Decoder Node Initialized in Shared Container.");
}

void DepthDecoderNode::tensor_callback(const isaac_ros_tensor_list_interfaces::msg::TensorList::SharedPtr msg)
{
  if (msg->tensors.empty()) return;

  // 1. Extract raw data from the TensorList
  const auto & raw_data = msg->tensors[0].data;
  
  // 2. Map to an OpenCV matrix (Zero-copy mapping at the CPU level)
  // Assuming 518x518 output from Depth Anything V2
  cv::Mat depth_mat(518, 518, CV_32FC1, (void*)raw_data.data());

  std_msgs::msg::Header header;
  header.stamp = this->now();
  header.frame_id = "camera_depth_optical_frame";

  // PATH A: THE MATH (Absolute Meters)
  // ==========================================
//  double scale_factor = 5.0; // Tune this to your physical camera
 // cv::Mat depth_meters;

  //depth_mat.convertTo(depth_meters, CV_32FC1, scale_factor);//
  int h = depth_mat.rows;
  int w = depth_mat.cols;

// 2. Calculate the exact bounding box using your Python fractions
  int x1 = static_cast<int>(w * 0.3475);
  int x2 = static_cast<int>(w * 0.695);
  int y1 = static_cast<int>(h * 0.483);
  int y2 = static_cast<int>(h * 0.73);

// 3. Define the Region of Interest (x, y, width, height) and crop instantaneously
  cv::Rect roi(x1, y1, x2 - x1, y2 - y1);
  cv::Mat cropped_depth = depth_mat(roi);

// 4. Publish ONLY the tiny cropped image to stop the ROS 2 network bottleneck
  sensor_msgs::msg::Image::SharedPtr math_msg = cv_bridge::CvImage(header, "32FC1", cropped_depth).toImageMsg();
  publisher_math_->publish(*math_msg);

  // 3. Find dynamic min and max for this specific frame
  double min_val, max_val;
  cv::minMaxLoc(depth_mat, &min_val, &max_val);

  // 4. Normalize to 0-255
  cv::Mat depth_normalized;
  if (max_val > min_val) {
    depth_mat.convertTo(depth_normalized, CV_8UC1, 255.0 / (max_val - min_val), -min_val * 255.0 / (max_val - min_val));
  } else {
    depth_normalized = cv::Mat::zeros(518, 518, CV_8UC1);
  }

  // 5. Apply the Heatmap Color
 // cv::Mat depth_colored;
  //cv::applyColorMap(depth_normalized, depth_colored, cv::COLORMAP_JET);

 

  sensor_msgs::msg::Image::SharedPtr img_msg = cv_bridge::CvImage(header, "mono8", depth_normalized).toImageMsg();
  publisher_.publish(img_msg);
}

}  // namespace custom_rover_nodes

// Register the component with class_loader.
// This is the magic line that allows it to be loaded into the NITROS ComposableNodeContainer.
RCLCPP_COMPONENTS_REGISTER_NODE(custom_rover_nodes::DepthDecoderNode)
