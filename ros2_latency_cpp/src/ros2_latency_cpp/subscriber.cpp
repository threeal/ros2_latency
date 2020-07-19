#include "ros2_latency_cpp/subscriber.hpp"

using namespace ros2_latency_cpp;
using namespace std::chrono_literals;

Subscriber::Subscriber(std::string node_name, std::string publisher_node_name)
  : rclcpp::Node(node_name),
    init_delay(0)
{
  // initialize the byte data subscription
  {
    using ByteData = ros2_latency_interfaces::msg::ByteData;
    byte_data_subscription = create_subscription<ByteData>(
      publisher_node_name + "/byte_data", 10,
      [this](std::unique_ptr<ByteData> message) {
        auto byte_size = message->data.size();
        auto delay = (now().seconds() * 1000.0) - message->timestamp;

        RCLCPP_INFO_STREAM(get_logger(), "receiving " << byte_size
          << " bytes of data in " << delay << " ms");
      }
    );

    RCLCPP_INFO_STREAM(get_logger(), "subscribing byte data on "
      << byte_data_subscription->get_topic_name());
  }

  // initialize the delay init client
  {
    using DelayInit = ros2_latency_interfaces::srv::DelayInit;
    delay_init_client = create_client<DelayInit>(publisher_node_name + "/init");
  }
}

bool Subscriber::start() {
  while (!delay_init_client->wait_for_service(3s)) {
    if (!rclcpp::ok()) {
      return false;
    }
  }

  using DelayInit = ros2_latency_interfaces::srv::DelayInit;
  auto request = std::make_shared<DelayInit::Request>();
  request->timestamp = now().seconds() * 1000.0;

  auto response_future = delay_init_client->async_send_request(request);
  if (rclcpp::spin_until_future_complete(shared_from_this(), response_future) ==
      rclcpp::executor::FutureReturnCode::SUCCESS) {
    auto response = response_future.get();
    init_delay = response->timestamp - request->timestamp;

    RCLCPP_INFO_STREAM(get_logger(), "init delay is " << init_delay << " ms");

    return true;
  }
  else {
    RCLCPP_ERROR_STREAM(get_logger(), "failed to call "
      << delay_init_client->get_service_name());

    return false;
  }
}