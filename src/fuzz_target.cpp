// This is the eloquent version (the API has tiny diffrences starting foxy)
// https://index.ros.org/doc/ros2/Tutorials/Writing-A-Simple-Cpp-Service-And-Client/

#include "rclcpp/rclcpp.hpp"
#include "example_interfaces/srv/add_two_ints.hpp"
#include "std_msgs/msg/string.hpp"

#include <iostream>
#include <chrono>
#include <cstdlib>
#include <memory>
#include <functional>
#include <string>


using namespace std::chrono_literals;

//
// Message generation API
//
bool getInt64(int64_t& out) {
    int bytes[8] = {};
    for (int i = 0; i < 8; ++i) {
        int c = getchar();
        if (c == EOF) {
            return false;
        }
        bytes[i] = c;
    }
    out = 0;
    for (int i = 0; i < 8; ++i)
        out += (bytes[i] << (i*8));

    return true;
}

bool getBool(bool& b) {
    int c = getchar();
    if (c == EOF) {
        return false;
    }
    b = (c % 2 == 0);
    return true;
}

bool getString(std::string& s, int8_t size) {
    std::string res(size, '\0' );
    if (fread(&res[0], sizeof(char), size, stdin) != (size_t) size) {
        return false;
    }
    s = res;
    return true;
}


//
// 1. Server
//
void fuzz_server() {
  std::shared_ptr<rclcpp::Node> node = 
    rclcpp::Node::make_shared("add_two_ints_client");

  rclcpp::Client<example_interfaces::srv::AddTwoInts>::SharedPtr client =
    node->create_client<example_interfaces::srv::AddTwoInts>("add_two_ints");

  long a, b;
  while (getInt64(a) && getInt64(b)) {
    auto request = 
      std::make_shared<example_interfaces::srv::AddTwoInts::Request>();
    request->a = a;
    request->b = b;

    while (!client->wait_for_service(1s)) {
      if (!rclcpp::ok()) {
        RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Interrupted while waiting for the service. Exiting.");
        return;
      }
      RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "service not available, waiting again...");
    }

    auto result = client->async_send_request(request);
    
    // Wait for the result.
    if (rclcpp::spin_until_future_complete (node, result) ==
      rclcpp::executor::FutureReturnCode::SUCCESS)
    {
      RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Sum: %ld", result.get()->sum);
    } 
    else 
    {
      RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Failed to call service add_two_ints");
    }
  }
}

//
// 2. Subscriber
//
class MinimalPublisher : public rclcpp::Node
{
  public:
    MinimalPublisher()
    : Node("minimal_publisher")
    {
      publisher_ = this->create_publisher<std_msgs::msg::String>("topic", 10);
      timer_ = this->create_wall_timer(
      1ms, std::bind(&MinimalPublisher::timer_callback, this));
    }

  private:
    void timer_callback()
    {
      std::string content;
      if (!getString(content, 3)) {
        rclcpp::shutdown();
        exit(EXIT_SUCCESS);
      }

      auto message = std_msgs::msg::String();
      message.data = content;
      RCLCPP_INFO(this->get_logger(), "Publishing: '%s'", message.data.c_str());
      publisher_->publish(message);
    }

    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
};

void fuzz_subscriber() {
  rclcpp::spin(std::make_shared<MinimalPublisher>());
}

//
// Main driver
//
int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);

  // Switch on the two fuzzing modes
  bool is_fuzzing_server;
  if (getBool(is_fuzzing_server)) {
    if (is_fuzzing_server) {
      fuzz_server();
    } else {
      fuzz_subscriber();
    }
  }
  
  rclcpp::shutdown();
  return 0;
}
