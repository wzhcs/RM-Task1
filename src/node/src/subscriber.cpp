#include "node/subscriber.hpp"
#include <algorithm> // std::find

SerialSubscriber::SerialSubscriber(const rclcpp::NodeOptions & options)
: Node("serial_subscriber_cpp", options),
  receive_buffer_()
{
  // 支持参数配置
  const std::string device_name = this->declare_parameter<std::string>("device_name", "/dev/pts/11");
  int baud_rate = this->declare_parameter<int>("baud_rate", 38400);

  // 串口配置
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

    RCLCPP_INFO(this->get_logger(), "Serial port opened: %s, baud_rate: %d", device_name.c_str(), baud_rate);
  }
  catch (const std::exception &ex)
  {
    RCLCPP_ERROR(this->get_logger(), "Failed to initialize serial port: %s", ex.what());
    rclcpp::shutdown();
    return;
  }

  // 发布者
  publisher_ = this->create_publisher<std_msgs::msg::String>("from_serial", 10);

  // 开启异步串口接收
  async_receive_message();
}

void SerialSubscriber::async_receive_message()
{
  auto port = serial_driver_->port();
  port->async_receive([this](const std::vector<uint8_t> &data, const size_t &size)
  {
    if (size > 0)
    {
      // 将收到的数据放入缓冲区
      receive_buffer_.insert(receive_buffer_.end(), data.begin(), data.begin() + size);

      // 解析完整帧，帧格式：0xAA ...payload... 0xBB
      while (true)
      {
        auto head_it = std::find(receive_buffer_.begin(), receive_buffer_.end(), 0xAA);
        if (head_it == receive_buffer_.end())
        {
          // 找不到帧头，清空缓冲区
          receive_buffer_.clear();
          break;
        }

        auto tail_it = std::find(head_it + 1, receive_buffer_.end(), 0xBB);
        if (tail_it == receive_buffer_.end())
        {
          // 找不到帧尾，保留帧头及其之后的数据，等待后续数据补全
          receive_buffer_.erase(receive_buffer_.begin(), head_it);
          break;
        }

        // 找到完整帧，有效负载是帧头和帧尾之间的数据
        std::string message(head_it + 1, tail_it);

        RCLCPP_INFO(this->get_logger(), "Received framed message: %s (%ld bytes)", message.c_str(), message.size());

        // 发布消息
        std_msgs::msg::String msg;
        msg.data = message;
        publisher_->publish(msg);

        // 移除已经处理的帧数据
        receive_buffer_.erase(receive_buffer_.begin(), tail_it + 1);
      }
    }

    // 持续监听
    async_receive_message();
  });
}

#include "rclcpp/rclcpp.hpp"

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<SerialSubscriber>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
