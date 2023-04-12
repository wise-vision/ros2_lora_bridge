#ifndef ROS2_LORA_TOPIC_BRIDGE_HPP_
#define ROS2_LORA_TOPIC_BRIDGE_HPP_

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "lora.h" // Include the LoRa library

class Ros2LoRaTopicBridge : public rclcpp::Node
{
public:
  Ros2LoRaTopicBridge();

private:
  void ros2TopicCallback(const std_msgs::msg::String::SharedPtr msg);
  void loraReceiveCallback(const std::vector<uint8_t> &data);

  LoRa *lora_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr ros2_publisher_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr ros2_subscriber_;
};

#endif /* ROS2_LORA_TOPIC_BRIDGE_HPP_ */

