#ifndef ROS2_LATENCY_CPP__SUBSCRIBER_HPP_
#define ROS2_LATENCY_CPP__SUBSCRIBER_HPP_

#include <rclcpp/rclcpp.hpp>

#include <ros2_latency_interfaces/msg/byte_data.hpp>
#include <ros2_latency_interfaces/srv/delay_init.hpp>

#include <memory>
#include <string>

namespace ros2_latency_cpp
{
  class Subscriber : public rclcpp::Node
  {
  public:

    Subscriber(std::string node_name, std::string publisher_node_name);

    bool start();

  private:

    double init_delay;

    std::shared_ptr<rclcpp::Subscription<ros2_latency_interfaces::msg::ByteData>>
      byte_data_subscription;

    std::shared_ptr<rclcpp::Client<ros2_latency_interfaces::srv::DelayInit>>
      delay_init_client;
  };
}

#endif