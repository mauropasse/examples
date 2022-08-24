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

#include <cinttypes>
#include <memory>

#include "example_interfaces/srv/add_two_ints.hpp"
#include "rclcpp/rclcpp.hpp"

using AddTwoInts = example_interfaces::srv::AddTwoInts;

void handle_service(
  const std::shared_ptr<rmw_request_id_t> request_header,
  const std::shared_ptr<AddTwoInts::Request> request,
  const std::shared_ptr<AddTwoInts::Response> response)
{
  (void)request_header;
  (void)request;
  (void)response;
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  auto executor = std::make_shared<rclcpp::executors::EventsExecutor>();

  std::thread my_thread( [&] () { executor->spin(); } );

  {
    rclcpp::NodeOptions node_options = rclcpp::NodeOptions();
    node_options.use_intra_process_comms(false);
    node_options.start_parameter_services(false);
    node_options.start_parameter_event_publisher(false);

    auto g_node = rclcpp::Node::make_shared("minimal_service", node_options);
    auto server = g_node->create_service<AddTwoInts>("add_two_ints", handle_service);
    executor->add_node(g_node);
    // server->set_on_new_request_callback([](size_t a){(void)a;});
    std::this_thread::sleep_for(std::chrono::seconds(1));
    std::cout << "DESTROY" << std::endl << std::endl << std::endl ;
  }

  rclcpp::shutdown();
  my_thread.join();
  return 0;
}

