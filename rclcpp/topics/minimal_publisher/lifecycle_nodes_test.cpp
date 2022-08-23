// Copyright 2016 Open Source Robotics Foundation, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <chrono>
#include <string>
#include <thread>

#include "rclcpp/executors.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "std_msgs/msg/string.hpp"
#include "lifecycle_msgs/msg/state.hpp"
#include "lifecycle_msgs/msg/transition.hpp"

using namespace std::chrono_literals;

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);

  // Create node 1
  auto n1 = std::make_shared<rclcpp_lifecycle::LifecycleNode>("node_1", rclcpp::NodeOptions());
  auto p1 = n1->create_publisher<std_msgs::msg::String>("topic", 10);
  auto t1 = n1->create_wall_timer(500ms,[&](){
    auto message = std_msgs::msg::String();
    message.data = "1";
    std::cout << message.data << std::endl;
    p1->publish(message);
  });

  // Create node 2
  auto n2 = std::make_shared<rclcpp_lifecycle::LifecycleNode>("node_2", rclcpp::NodeOptions());
  auto p2 = n2->create_publisher<std_msgs::msg::String>("topic", 10);
  auto t2 = n2->create_wall_timer(500ms,[&](){
    auto message = std_msgs::msg::String();
    message.data = " 2";
    std::cout <<  message.data << std::endl;
    p2->publish(message);
  });

  // Create node 3
  auto n3 = std::make_shared<rclcpp_lifecycle::LifecycleNode>("node_3", rclcpp::NodeOptions());
  auto p3 = n3->create_publisher<std_msgs::msg::String>("topic", 10);
  auto t3 = n3->create_wall_timer(500ms,[&](){
    auto message = std_msgs::msg::String();
    message.data = "  3";
    std::cout << message.data << std::endl;
    p3->publish(message);
  });

  // Create node 4 (subscription)
  auto n4 = std::make_shared<rclcpp_lifecycle::LifecycleNode>("node_4", rclcpp::NodeOptions());
  auto s4 = n2->create_subscription<std_msgs::msg::String>(
    "topic",
    10,
    [](const std_msgs::msg::String & msg){
      std::cout <<  msg.data  << std::endl << std::endl;
      });

  // Create executor
  auto ex = std::make_shared<rclcpp::executors::EventsExecutor>();

  ex->add_node(n1->get_node_base_interface());
  ex->add_node(n2->get_node_base_interface());
  ex->add_node(n3->get_node_base_interface());
  ex->add_node(n4->get_node_base_interface());

  p1->on_activate();
  p2->on_activate();
  p3->on_activate();

  std::thread my_thread([&](){
    ex->spin();
  });

  std::this_thread::sleep_for(std::chrono::seconds(3));
  std::cout <<  "t3.reset();" << std::endl;
  t3->cancel();
  t3.reset();
  p3.reset();

  std::this_thread::sleep_for(std::chrono::seconds(3));
  std::cout <<  "t2.reset();" << std::endl;
  t2->cancel();
  t2.reset();
  p2.reset();

  std::this_thread::sleep_for(std::chrono::seconds(3));
  std::cout <<  "t1.reset();" << std::endl;
  t1->cancel();
  t1.reset();
  p1.reset();

  rclcpp::shutdown();

  my_thread.join();

  return 0;
}
