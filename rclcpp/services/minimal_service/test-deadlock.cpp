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

#include "rclcpp/rclcpp.hpp"

#include <std_srvs/srv/set_bool.hpp>
#include "rclcpp/executors/events_executor/events_executor.hpp"


#include <string>

using namespace std::chrono;

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  auto service_executor = std::make_shared<rclcpp::executors::EventsExecutor>(); //EventsExecutor

  auto client_executor = std::make_shared<rclcpp::executors::EventsExecutor>();

  // Create a node
  auto opt = rclcpp::NodeOptions();
  auto server_node = rclcpp::Node::make_shared("server_node", opt);
  service_executor->add_node(server_node->get_node_base_interface());
  auto m_service = rclcpp::create_service<std_srvs::srv::SetBool>(
      server_node->get_node_base_interface(),
      server_node->get_node_services_interface(),
      "example_service",
      [server_node](const std::shared_ptr<std_srvs::srv::SetBool::Request> request,
          const std::shared_ptr<std_srvs::srv::SetBool::Response> response) {
          static unsigned int serial_num = 1;
          (void)request;
          RCLCPP_INFO(server_node->get_logger(), "Received service client request... Sending response: %d", serial_num);
          response->success = true;
          response->message = std::to_string(serial_num++);
      },
      rclcpp::ServicesQoS().get_rmw_qos_profile(),
      nullptr);

  auto client_node = rclcpp::Node::make_shared("client_node", opt);
  client_executor->add_node(client_node);
  auto m_client = client_node->create_client<std_srvs::srv::SetBool>("example_service");

  auto service_spin_thread = std::thread(std::bind([&]()
  {
      service_executor->spin();
  }));

  auto client_spin_thread = std::thread(std::bind([&]()
  {
      client_executor->spin();
  }));


  //////////////////////   TEST   ///////////////////////////////
  auto request = std::make_shared<std_srvs::srv::SetBool::Request>();
  request->data = true;

  while (!m_client->wait_for_service(1s)) {
      if (!rclcpp::ok()) {
      RCLCPP_ERROR(client_node->get_logger(), "Interrupted while waiting for the service. Exiting.");
      return 0;
      }
      RCLCPP_INFO(client_node->get_logger(), "service not available, waiting again...");
  }
  uint counter = 1;
  do {
      RCLCPP_INFO(client_node->get_logger(), "Sending request: %d" , counter);
      auto result_future = m_client->async_send_request(request);
      RCLCPP_INFO(client_node->get_logger(), "Waiting for response: %d" , counter);

      // if (client_executor->spin_until_future_complete(result_future, std::chrono::milliseconds(10)) !=
      //   rclcpp::FutureReturnCode::SUCCESS)
      // {
      //   std::this_thread::sleep_for(std::chrono::seconds(2));
      //   RCLCPP_ERROR(client_node->get_logger(), "service call failed :(");
      //   m_client->remove_pending_request(result_future);
      // }

      auto answer = result_future.get();

      assert(answer->success);
      RCLCPP_INFO(client_node->get_logger(), "Got response: %s", answer->message.c_str());
  }while(++counter<1000000);


  rclcpp::shutdown();
  service_spin_thread.join();
  client_spin_thread.join();
  return 0;
}
