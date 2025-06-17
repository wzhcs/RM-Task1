#ifndef SERIAL_SUBSCRIBER_HPP
#define SERIAL_SUBSCRIBER_HPP

#include "rclcpp/rclcpp.hpp"
#include "serial_driver/serial_driver.hpp"
#include <string>
#include <vector>
#include <memory>
#include "std_msgs/msg/string.hpp"

class SerialSubscriber : public rclcpp::Node
{
public:
  explicit SerialSubscriber(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

private:
  void async_receive_message();

  std::shared_ptr<drivers::serial_driver::SerialDriver> serial_driver_;
  std::shared_ptr<drivers::common::IoContext> io_context_;

  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;

  // 添加帧接收缓冲区
  std::vector<uint8_t> receive_buffer_;
};

#endif // SERIAL_SUBSCRIBER_HPP


