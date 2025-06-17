#ifndef SERIAL_NODE_HPP
#define SERIAL_NODE_HPP

#include "rclcpp/rclcpp.hpp"
#include "serial_driver/serial_driver.hpp"
#include <string>
#include <vector>
#include <memory>

class SerialNode : public rclcpp::Node
{
public:
  explicit SerialNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

private:
  void send_message_timer_callback();
  void async_receive_message();

  std::shared_ptr<drivers::serial_driver::SerialDriver> serial_driver_;
  std::shared_ptr<drivers::common::IoContext> io_context_;
  rclcpp::TimerBase::SharedPtr transmit_timer_;

  std::vector<uint8_t> transmit_data_buffer;
  std::vector<uint8_t> receive_buffer_;  // 帧接收缓冲区

  std::string send_message_;  
};

#endif // SERIAL_NODE_HPP



