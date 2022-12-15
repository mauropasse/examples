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
#include <thread>

#include "rclcpp/executors.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "rclcpp/executors/events_executor/events_executor.hpp"

using namespace std::chrono_literals;

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);

  auto ex = std::make_shared<rclcpp::executors::EventsExecutor>();

  std::thread my_thread([&](){ ex->spin(); });

  rclcpp::NodeOptions node_options = rclcpp::NodeOptions();
  node_options.enable_rosout(false);
  node_options.use_intra_process_comms(true);
  node_options.start_parameter_services(true);
  node_options.start_parameter_event_publisher(true);

  {
    auto n1 = std::make_shared<rclcpp_lifecycle::LifecycleNode>("node_1", node_options);
    ex->add_node(n1->get_node_base_interface());
    std::this_thread::sleep_for(100ms);
  }

  rclcpp::shutdown();
  my_thread.join();
  return 0;
}
