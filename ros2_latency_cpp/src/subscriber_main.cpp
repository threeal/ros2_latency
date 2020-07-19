#include <rclcpp/rclcpp.hpp>
#include <ros2_latency_cpp/subscriber.hpp>

#include <memory>

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);

  auto subscriber = std::make_shared<ros2_latency_cpp::Subscriber>(
    "subscriber", "publisher"
  );

  if (subscriber->start()) {
    rclcpp::spin(subscriber);
  }

  rclcpp::shutdown();

  return 0;
}