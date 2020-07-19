#include "ros2_latency_cpp/byte_publisher.hpp"

using namespace ros2_latency_cpp;
using namespace std::chrono_literals;

BytePublisher::BytePublisher(std::string node_name)
  : rclcpp::Node(node_name),
    data_size(0)
{
  // initialize the byte data publisher
  {
    using ByteData = ros2_latency_interfaces::msg::ByteData;
    byte_data_publisher = create_publisher<ByteData>(node_name + "/byte_data", 10);
  }

  // initialize the delay init service
  {
    using DelayInit = ros2_latency_interfaces::srv::DelayInit;
    delay_init_service = create_service<DelayInit>(
      node_name + "/init",
      [this](std::shared_ptr<DelayInit::Request> request,
          std::shared_ptr<DelayInit::Response> response) {
        response->timestamp = now().seconds() * 1000.0;

        RCLCPP_INFO_STREAM(get_logger(), "init delay is "
          << response->timestamp - request->timestamp << " ms");

        // initialize the timer
        {
          data_size = 1;
          if (!publish_timer) {
            publish_timer = this->create_wall_timer(30ms, [this]() {
              ros2_latency_interfaces::msg::ByteData message;

              message.data.resize(data_size);
              for (int i = 0; i < data_size; ++i) {
                message.data[i] = i;
              }

              data_size *= 2;
              if (data_size > 10000000) {
                publish_timer->cancel();
              }

              message.timestamp = now().seconds() * 1000.0;
              byte_data_publisher->publish(message);

              RCLCPP_INFO_STREAM(get_logger(), "publishing "
                << message.data.size() << " bytes of data");
            });
          }
          else {
            publish_timer->reset();
          }
        }
      }
    );

    RCLCPP_INFO_STREAM(get_logger(), "start the test by triggering "
      << delay_init_service->get_service_name());
  }
}