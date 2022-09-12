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
#include <chrono>
#include <csignal>

// Generate an interrupt
// std::raise(SIGINT);
using namespace std::chrono_literals;

#include "example_interfaces/srv/add_two_ints.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp/executors/events_executor/events_executor.hpp"

// #include "rclcpp/experimental/buffers/simple_events_queue.hpp"
// #include "rclcpp/experimental/buffers/lock_free_events_queue.hpp"
// #include "rclcpp/experimental/buffers/bounded_events_queue.hpp"
// #include "rclcpp/experimental/buffers/waitset_events_queue.hpp"
using Executor = rclcpp::executors::EventsExecutor;
// using Executor = rclcpp::executors::StaticSingleThreadedExecutor;

using AddTwoInts = example_interfaces::srv::AddTwoInts;
rclcpp::Node::SharedPtr service_node = nullptr;
rclcpp::Node::SharedPtr client_node = nullptr;


void handle_service(
  const std::shared_ptr<rmw_request_id_t> request_header,
  const std::shared_ptr<AddTwoInts::Request> request,
  const std::shared_ptr<AddTwoInts::Response> response)
{
  (void)request_header;
  response->sum = request->a + request->b;
  RCLCPP_INFO(
    service_node->get_logger(),
    "Handle service. Request: %" PRId64 " + %" PRId64 " = %" PRId64, request->a, request->b, response->sum);
}

void handle_servicio(
  const std::shared_ptr<rmw_request_id_t> request_header,
  const std::shared_ptr<AddTwoInts::Request> request,
  const std::shared_ptr<AddTwoInts::Response> response)
{
  (void)request_header;
  response->sum = request->a * request->b;
  RCLCPP_INFO(
    service_node->get_logger(),
    "Handle servicio. Request: %" PRId64 " * %" PRId64 " = %" PRId64, request->a, request->b, response->sum);
}

void client_callback(
  std::shared_ptr<typename AddTwoInts::Request> request,
  typename rclcpp::Client<AddTwoInts>::SharedFuture result_future)
{
    auto result = result_future.get();
    RCLCPP_INFO(
      client_node->get_logger(),
      "Client callback. Result of %" PRId64 " + %" PRId64 " = %" PRId64, request->a, request->b, result->sum);
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  // Node options
  rclcpp::NodeOptions node_options = rclcpp::NodeOptions();
  node_options.use_intra_process_comms(false);
  node_options.start_parameter_services(false);
  node_options.start_parameter_event_publisher(false);

  // Create nodes with node options
  service_node = rclcpp::Node::make_shared("service", node_options);
  client_node = rclcpp::Node::make_shared("client", node_options);

  // Client/Service options
  rclcpp::QoSInitialization qos_init(RMW_QOS_POLICY_HISTORY_KEEP_LAST, 4);

  rmw_qos_profile_t rmw_qos_profile = rmw_qos_profile_services_default;
  rmw_qos_profile.history = RMW_QOS_POLICY_HISTORY_KEEP_LAST;
  std::cout << "Setting OoS depth = 4 for client" << std::endl;
  rmw_qos_profile.depth = 4;

  rclcpp::QoS qos_profile(qos_init, rmw_qos_profile);

  // Create client and service with options
  auto client = client_node->create_client<AddTwoInts>("add_two_ints", qos_profile, nullptr, rclcpp::IntraProcessSetting::Enable);
  // auto client2 = client_node->create_client<AddTwoInts>("add_two_ints", qos_profile, nullptr, rclcpp::IntraProcessSetting::Disable);

  std::cout << "Setting OoS depth = 4 for service" << std::endl;
  rmw_qos_profile.depth = 4;
  auto server = service_node->create_service<AddTwoInts>("add_two_ints", handle_service, qos_profile, nullptr, rclcpp::IntraProcessSetting::Enable);
  // auto server2 = service_node->create_service<AddTwoInts>("add_two_ints", handle_servicio, qos_profile);

  // auto client_qos = client->get_actual_qos();
  // std::cout << "Client OoS depth: " << client_qos.depth() << std::endl;

  // auto server_qos = server->get_actual_qos();
  // std::cout << "Server OoS depth: " << server_qos.depth() << std::endl;

  // Create separate executors for client and service nodes to better show the issue
  // auto service_queue = std::make_unique<rclcpp::experimental::buffers::BoundedEventsQueue>();
  // auto service_queue = std::make_unique<rclcpp::experimental::buffers::LockFreeEventsQueue>();
  // auto service_queue = std::make_unique<rclcpp::experimental::buffers::WaitSetEventsQueue>();
  // auto service_executor = std::make_shared<Executor>(std::move(service_queue));
  auto service_executor = std::make_shared<Executor>();
  service_executor->add_node(service_node);


  // auto client_queue = std::make_unique<rclcpp::experimental::buffers::BoundedEventsQueue>();
  // auto client_queue = std::make_unique<rclcpp::experimental::buffers::LockFreeEventsQueue>();
  // auto client_queue = std::make_unique<rclcpp::experimental::buffers::WaitSetEventsQueue>();
  // auto client_executor = std::make_shared<Executor>(std::move(client_queue));
  auto client_executor = std::make_shared<Executor>();
  client_executor->add_node(client_node);

  char a;
  std::cout << "Press key to spin the Server" << std::endl;
  std::cin >> a;

  /******** SERVICE EXECUTOR THREAD ************************************************************/
  std::thread service_spin_thread([=](){
      std::cout << "service_executor->spin()" << std::endl;
      service_executor->spin();
  });
  service_spin_thread.detach();
  /********************************************************************************************/

  std::cout << "Press key to spin the Client" << std::endl;
  std::cin >> a;

  /******** CLIENT EXECUTOR THREAD ************************************************************/
  std::thread spin_thread([=](){
      std::cout << "client_executor->spin()" << std::endl;
      client_executor->spin();
  });
  spin_thread.detach();
  /********************************************************************************************/

  // Send requests (depth is set to 1, so we should only have a single client request)
  for (int i=0; i <= 5; i++) {
    AddTwoInts::Request::SharedPtr request = std::make_shared<AddTwoInts::Request>();
    request->a = i;
    request->b = i;

    std::function<void(typename rclcpp::Client<AddTwoInts>::SharedFuture future)> callback_function = std::bind(
        &client_callback,
        request,
        std::placeholders::_1
    );

    std::cout << "send request number: " << i << std::endl;
    client->async_send_request(request, callback_function);

    // AddTwoInts::Request::SharedPtr request2 = std::make_shared<AddTwoInts::Request>();
    // request2->a = i+3;
    // request2->b = i+3;
    // std::function<void(typename rclcpp::Client<AddTwoInts>::SharedFuture future)> callback_function2 = std::bind(
    //     &client_callback,
    //     request2,
    //     std::placeholders::_1
    // );
    // client2->async_send_request(request2, callback_function2);

    std::this_thread::sleep_for(10ms);
  }


  std::cin >> a;

  rclcpp::shutdown();
  service_node = nullptr;
  client_node = nullptr;
  return 0;
}

