#include <rclcpp/rclcpp.hpp>
#include <ros2_latency_cpp/byte_publisher.hpp>

#include <memory>

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);

  auto publisher = std::make_shared<ros2_latency_cpp::BytePublisher>("publisher");

  rclcpp::spin(publisher);

  rclcpp::shutdown();

  return 0;
}