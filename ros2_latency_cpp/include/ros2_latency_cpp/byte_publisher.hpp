#ifndef ROS2_LATENCY_CPP__BYTE_PUBLISHER_HPP_
#define ROS2_LATENCY_CPP__BYTE_PUBLISHER_HPP_

#include <rclcpp/rclcpp.hpp>
#include <ros2_latency_interfaces/msg/byte_data.hpp>
#include <ros2_latency_interfaces/srv/delay_init.hpp>

#include <memory>
#include <string>

namespace ros2_latency_cpp
{
  class BytePublisher : public rclcpp::Node
  {
  public:

    BytePublisher(std::string node_name);

  private:

    int data_size;

    std::shared_ptr<rclcpp::Publisher<ros2_latency_interfaces::msg::ByteData>>
      byte_data_publisher;

    std::shared_ptr<rclcpp::Service<ros2_latency_interfaces::srv::DelayInit>>
      delay_init_service;

    std::shared_ptr<rclcpp::TimerBase> publish_timer;
  };
}

#endif