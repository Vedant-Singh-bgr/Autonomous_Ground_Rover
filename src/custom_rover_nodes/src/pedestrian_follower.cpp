#include "custom_rover_nodes/pedestrian_follower.hpp"
#include "rclcpp_components/register_node_macro.hpp"
#include <cv_bridge/cv_bridge.h>
namespace custom_rover_nodes
{

PedestrianFollower::PedestrianFollower(const rclcpp::NodeOptions & options)
: Node("PedestrianFollower", options)
{
  rclcpp::SubscriptionOptions sub_options;
  sub_options.use_intra_process_comm = rclcpp::IntraProcessSetting::Enable;

  detection_sub_.subscribe(this, "/pedestrian_model/detections", rmw_qos_profile_default, sub_options);
  depth_sub_.subscribe(this, "/depth_tensor_output", rmw_qos_profile_default, sub_options);
  rclcpp::PublisherOptions pub_options;
  pub_options.use_intra_process_comm = rclcpp::IntraProcessSetting::Enable;
  publisher_math_ = this->create_publisher<sensor_msgs::msg::Image>("/camera/depth/pedestrian_raw_disparity", 10, pub_options);
  distance_pub_ = this->create_publisher<std_msgs::msg::Float32>("/rover/target_disparity", 10);
  // Change how the publisher is created:
  target_pub_ = this->create_publisher<geometry_msgs::msg::Point>("/rover/target_tracking", 10);
  sync_ = std::make_shared<message_filters::Synchronizer<SyncPolicy>>(
    SyncPolicy(10), detection_sub_, depth_sub_);
  sync_->registerCallback(
    std::bind(&PedestrianFollower::synced_callback, this, std::placeholders::_1, std::placeholders::_2)
  );
  
  
  
  RCLCPP_INFO(this->get_logger(), "C++ Time Synchronizer Initialized with Zero-Copy IPC.");
}

void PedestrianFollower::synced_callback(
  const vision_msgs::msg::Detection2DArray::ConstSharedPtr& detection_msg,
  const isaac_ros_tensor_list_interfaces::msg::TensorList::ConstSharedPtr& depth_msg)
{
    if ((detection_msg->detections.empty())||(depth_msg->tensors.empty())) {
    return; // No pedestrians found in this matched frame
  }
  int center_x = static_cast<int>((detection_msg->detections[0].bbox.center.position.x)*0.863333);
  int center_y = static_cast<int>((detection_msg->detections[0].bbox.center.position.y)*1.295);
  int width = static_cast<int>((detection_msg->detections[0].bbox.size_x)*0.863333);
  int height = static_cast<int>((detection_msg->detections[0].bbox.size_y)*1.295);
  int w = static_cast<int>(width*0.4);
  int h = static_cast<int>(height*0.3);
  int x1 = static_cast<int>(center_x - width*0.2);
  int y1 = static_cast<int>(center_y - height*0.3);

  const auto & raw_data = depth_msg->tensors[0].data;
  cv::Mat depth_mat(518, 518, CV_32FC1, (void*)raw_data.data());
  std_msgs::msg::Header header;
  header.stamp = this->now();
  header.frame_id = "camera_depth_optical_frame";
  cv::Rect desired_roi(x1, y1, w, h);
  cv::Rect image_bounds(0, 0, 518, 518);
  cv::Rect roi = desired_roi & image_bounds;
  if (roi.area() <= 0) {
    return; 
  }
  cv::Mat cropped_depth = depth_mat(roi);
  /*sensor_msgs::msg::Image::SharedPtr math_msg = cv_bridge::CvImage(header, "32FC1", cropped_depth).toImageMsg();
  publisher_math_->publish(*math_msg);*/
  // 1. Flatten the cropped matrix into a 1D vector of floats
  std::vector<float> depths;
  if (cropped_depth.isContinuous()) {
    depths.assign((float*)cropped_depth.datastart, (float*)cropped_depth.dataend);
  } else {
    for (int i = 0; i < cropped_depth.rows; ++i) {
      depths.insert(depths.end(), cropped_depth.ptr<float>(i), cropped_depth.ptr<float>(i) + cropped_depth.cols);
    }
  }

  // 2. Remove invalid sensor readings (0.0 or negative)
  depths.erase(std::remove_if(depths.begin(), depths.end(), [](float v){ return v <= 0.01f; }), depths.end());

  // 3. Find the Median Distance (Ignores noise/glitches)
  if (!depths.empty()) {
    std::nth_element(depths.begin(), depths.begin() + depths.size() / 2, depths.end());
    float median_distance = depths[depths.size() / 2];
    
    RCLCPP_INFO(this->get_logger(), "TARGET LOCKED: Pedestrian is %.2f disparity away", median_distance);
    // std_msgs::msg::Float32 distance_msg;
    // distance_msg.data = median_distance;
    // distance_pub_->publish(distance_msg);
    geometry_msgs::msg::Point tracking_msg;
    tracking_msg.x = static_cast<float>(center_x); // Horizontal pixel position
    tracking_msg.y = 0.0;                          // Not used
    tracking_msg.z = median_distance;              // Distance in meters
    
    target_pub_->publish(tracking_msg);
    
    // TO-DO: Publish this float to your rover control node!
  } else {
    RCLCPP_WARN(this->get_logger(), "No valid depth data in torso crop.");
  }


}


}  // namespace custom_rover_nodes

// Register the component with class_loader.
// This is the magic line that allows it to be loaded into the NITROS ComposableNodeContainer.
RCLCPP_COMPONENTS_REGISTER_NODE(custom_rover_nodes::PedestrianFollower)
