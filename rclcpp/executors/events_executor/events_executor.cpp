#include "rclcpp/rclcpp.hpp"
#include "rclcpp/executors/events_executor/events_executor.hpp"

using namespace std::chrono_literals;

int main(int argc, char * argv[])
{
  rclcpp::init(0, nullptr);

  auto node = std::make_shared<rclcpp::Node>("my_node");
  auto executor = std::make_shared<rclcpp::executors::EventsExecutor>();
  executor->add_node(node);

  rclcpp::shutdown();
  return 0;
}
