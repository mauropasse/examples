#include <memory>
#include "rclcpp/rclcpp.hpp"
#include "example_interfaces/srv/big_srv.hpp"
#include "rclcpp/experimental/executors/events_executor/events_executor.hpp"

#include <string>

using namespace std::chrono;
using BigSrv = example_interfaces::srv::BigSrv;
using rclcpp::experimental::executors::EventsExecutor;


int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  // Node options
  rclcpp::NodeOptions node_options = rclcpp::NodeOptions();
  node_options.use_intra_process_comms(false);
  node_options.start_parameter_services(false);
  node_options.start_parameter_event_publisher(false);

  // Create executors
  auto service_executor = std::make_shared<rclcpp::experimental::executors::EventsExecutor>();
  auto client_executor = std::make_shared<rclcpp::experimental::executors::EventsExecutor>();

  // Create a node for the service
  auto server_node = rclcpp::Node::make_shared("server_node", node_options);
  service_executor->add_node(server_node->get_node_base_interface());

  // Create service
  auto m_service = rclcpp::create_service<BigSrv>(
      server_node->get_node_base_interface(),
      server_node->get_node_services_interface(),
      "example_service",
      [server_node](const std::shared_ptr<BigSrv::Request> request,
          const std::shared_ptr<BigSrv::Response> response)
      {
          static unsigned int serial_num = 1;
          RCLCPP_INFO(server_node->get_logger(), "Received service client request... Sending response: %d", serial_num);
          response->success = true;
          response->message = std::to_string(serial_num++);
          response->big_data_response = request->big_data_request;
      },
      rclcpp::ServicesQoS().get_rmw_qos_profile(),
      nullptr);

  // Create nodes for the clients
  auto client_node = rclcpp::Node::make_shared("client_node", node_options);
  auto client_node2 = rclcpp::Node::make_shared("client_node2", node_options);
  client_executor->add_node(client_node);
  client_executor->add_node(client_node2);

  // Create 2 clients with same service name
  auto m_client = client_node->create_client<BigSrv>("example_service");
  auto m_client_2 = client_node2->create_client<BigSrv>("example_service");

  // Spin the executors
  auto service_spin_thread = std::thread(std::bind([&](){ service_executor->spin(); }));
  auto client_spin_thread = std::thread(std::bind([&](){ client_executor->spin(); }));

  // Wait for service
  while (!m_client->wait_for_service(1s)) {
      if (!rclcpp::ok()) {
        RCLCPP_ERROR(client_node->get_logger(), "Interrupted while waiting for the service. Exiting.");
        return 0;
      }
      RCLCPP_INFO(client_node->get_logger(), "service not available, waiting again...");
  }

  // Make client requests in a loop many times
  uint counter = 1;
  do {
      auto request = std::make_shared<BigSrv::Request>();
      request->data = true;
      request->big_data_request[1000] = 3;

      RCLCPP_INFO(client_node->get_logger(), "Sending request: %d" , counter);
      auto result_future = m_client->async_send_request(request);

      RCLCPP_INFO(client_node->get_logger(), "Waiting for response: %d" , counter);

      auto answer = result_future.get();
      RCLCPP_INFO(client_node->get_logger(), "Got response: %d", answer->big_data_response[1000]);
  }while(++counter<10);

  rclcpp::shutdown();
  service_spin_thread.join();
  client_spin_thread.join();
  return 0;
}
