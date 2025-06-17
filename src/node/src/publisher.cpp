#include "node/publisher.hpp"
#include <chrono>
#include <algorithm> // for std::find

SerialNode::SerialNode(const rclcpp::NodeOptions & options)
  : Node("serial_node_cpp", options),
    transmit_data_buffer(),
    receive_buffer_()
{
  // 支持参数化配置
  std::string device_name = this->declare_parameter<std::string>("device_name", "/dev/pts/10");
  int baud_rate = this->declare_parameter<int>("baud_rate", 38400);

  // 新增：读取发送内容参数
  send_message_ = this->declare_parameter<std::string>("send_message", "Hello RM!");

  RCLCPP_INFO(this->get_logger(), "Serial port Node Open!");

  drivers::serial_driver::SerialPortConfig config(
      baud_rate,
      drivers::serial_driver::FlowControl::NONE,
      drivers::serial_driver::Parity::NONE,
      drivers::serial_driver::StopBits::ONE);

  try
  {
    io_context_ = std::make_shared<drivers::common::IoContext>(1);
    serial_driver_ = std::make_shared<drivers::serial_driver::SerialDriver>(*io_context_);
    serial_driver_->init_port(device_name, config);
    serial_driver_->port()->open();

    RCLCPP_INFO(this->get_logger(), "Serial port initialized successfully");
    RCLCPP_INFO(this->get_logger(), "Using device: %s", serial_driver_->port()->device_name().c_str());
    RCLCPP_INFO(this->get_logger(), "Baud_rate: %d", config.get_baud_rate());
  }
  catch (const std::exception &ex)
  {
    RCLCPP_ERROR(this->get_logger(), "Failed to initialize serial port: %s", ex.what());
    rclcpp::shutdown();
    return;
  }

  transmit_timer_ = this->create_wall_timer(
      std::chrono::milliseconds(5),
      std::bind(&SerialNode::send_message_timer_callback, this));

  async_receive_message();
}

void SerialNode::send_message_timer_callback()
{
  // 使用从参数读取的发送内容
  const std::string payload = send_message_;

  // 构造带帧头帧尾的数据包
  transmit_data_buffer.clear();
  transmit_data_buffer.push_back(0xAA); // 帧头
  transmit_data_buffer.insert(transmit_data_buffer.end(), payload.begin(), payload.end());
  transmit_data_buffer.push_back(0xBB); // 帧尾

  auto port = serial_driver_->port();

  try
  {
    size_t bytes_transmit_size = port->send(transmit_data_buffer);
    RCLCPP_INFO(this->get_logger(), "Transmitted framed message: %ld bytes", bytes_transmit_size);
  }
  catch (const std::exception &ex)
  {
    RCLCPP_ERROR(this->get_logger(), "Error Transmitting from serial port: %s", ex.what());
  }
}

void SerialNode::async_receive_message()
{
  auto port = serial_driver_->port();

  port->async_receive([this](const std::vector<uint8_t> &data, const size_t &size)
  {
    if (size > 0)
    {
      receive_buffer_.insert(receive_buffer_.end(), data.begin(), data.begin() + size);

      while (true)
      {
        auto head_it = std::find(receive_buffer_.begin(), receive_buffer_.end(), 0xAA);
        if (head_it == receive_buffer_.end())
        {
          receive_buffer_.clear();
          break;
        }

        auto tail_it = std::find(head_it + 1, receive_buffer_.end(), 0xBB);
        if (tail_it == receive_buffer_.end())
        {
          receive_buffer_.erase(receive_buffer_.begin(), head_it);
          break;
        }

        std::string message(head_it + 1, tail_it);
        RCLCPP_INFO(this->get_logger(), "Received framed message: %s (%ld bytes)", message.c_str(), message.size());

        receive_buffer_.erase(receive_buffer_.begin(), tail_it + 1);
      }
    }

    async_receive_message();
  });
}

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<SerialNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}

