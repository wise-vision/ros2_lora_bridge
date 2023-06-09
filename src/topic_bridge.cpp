#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "lora.h" // Include the LoRa library

class Ros2LoRaTopicBridge : public rclcpp::Node
{
public:
  Ros2LoRaTopicBridge() : Node("ros2_lora_topic_bridge")
  {
    // Set LoRa parameters
    double frequency = 915.0;
    int spreading_factor = 7;
    int coding_rate = 5;
    int bw = 125000;
    int sync_word = 0x34;
    int tx_power = 17;
    // Initialize LoRa
    lora_ = new LoRa(frequency, spreading_factor, coding_rate, bw, sync_word, tx_power);

    // ROS2 Publisher
    ros2_publisher_ = this->create_publisher<std_msgs::msg::String>("/lora_topic", 10);

    // ROS2 Subscriber
    ros2_subscriber_ = this->create_subscription<std_msgs::msg::String>(
        "/ros2_topic", 10,
        std::bind(&Ros2LoRaTopicBridge::ros2TopicCallback, this, std::placeholders::_1));

    // LoRa Receive Callback
    lora_->setReceiveCallback(
        std::bind(&Ros2LoRaTopicBridge::loraReceiveCallback, this, std::placeholders::_1));
  }

private:
  void ros2TopicCallback(const std_msgs::msg::String::SharedPtr msg)
  {
    // Convert ROS2 message to string
    std::string data = msg->data;
    // Send data over LoRa
    lora_->send(data);
  }

  void loraReceiveCallback(const std::vector<uint8_t> &data)
  {
    // Convert LoRa data to string
    std::string received_data(data.begin(), data.end());
    // Publish data on ROS2 topic
    auto ros2_msg = std::make_shared<std_msgs::msg::String>();
    ros2_msg->data = received_data;
    ros2_publisher_->publish(ros2_msg);
  }

  LoRa *lora_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr ros2_publisher_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr ros2_subscriber_;
};

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<Ros2LoRaTopicBridge>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
