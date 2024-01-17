// Copyright 2018 Open Source Robotics Foundation, Inc.
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
#include <cinttypes>
#include <memory>
#include <thread>
#include "rclcpp/executors/events_executor/events_executor.hpp"


#include "example_interfaces/action/fibonacci.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

using Fibonacci = example_interfaces::action::Fibonacci;
using GoalHandleFibonacci = rclcpp_action::ServerGoalHandle<Fibonacci>;
using ActionGoalHandle = rclcpp_action::ClientGoalHandle<Fibonacci>;

// Server: Handle goal callback
rclcpp_action::GoalResponse handle_goal(
  const rclcpp_action::GoalUUID & uuid, std::shared_ptr<const Fibonacci::Goal> goal)
{
  RCLCPP_INFO(rclcpp::get_logger("server"), "Got goal request with order %d", goal->order);
  (void)uuid;
  // Let's reject sequences that are over 5
  if (goal->order > 7) {
    return rclcpp_action::GoalResponse::REJECT;
  }
  return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse handle_cancel(
  const std::shared_ptr<GoalHandleFibonacci> goal_handle)
{
  RCLCPP_INFO(rclcpp::get_logger("server"), "Got request to cancel goal");
  (void)goal_handle;
  return rclcpp_action::CancelResponse::ACCEPT;
}

// Server: Execute goal callback
void execute(
  const std::shared_ptr<GoalHandleFibonacci> goal_handle)
{
  RCLCPP_INFO(rclcpp::get_logger("server"), "Executing goal");
  rclcpp::Rate loop_rate(1);
  const auto goal = goal_handle->get_goal();
  auto feedback = std::make_shared<Fibonacci::Feedback>();
  auto & sequence = feedback->sequence;
  sequence.push_back(0);
  sequence.push_back(1);
  auto result = std::make_shared<Fibonacci::Result>();

  for (int i = 1; (i < goal->order) && rclcpp::ok(); ++i) {
    // Check if there is a cancel request
    if (goal_handle->is_canceling()) {
      result->sequence = sequence;
      goal_handle->canceled(result);
      RCLCPP_INFO(rclcpp::get_logger("server"), "Goal Canceled");
      return;
    }
    // Update sequence
    sequence.push_back(sequence[i] + sequence[i - 1]);
    // Publish feedback
    goal_handle->publish_feedback(feedback);
    RCLCPP_INFO(rclcpp::get_logger("server"), "Publish Feedback");

    loop_rate.sleep();
  }

  // Check if goal is done
  if (rclcpp::ok()) {
    result->sequence = sequence;
    std::cout << "[MAUROS2] - Application call: goal_handle->succeed(result);" << std::endl;
    goal_handle->succeed(result);
    RCLCPP_INFO(rclcpp::get_logger("server"), "Goal Succeeded");
  }
}

void handle_accepted(const std::shared_ptr<GoalHandleFibonacci> goal_handle)
{
  // this needs to return quickly to avoid blocking the executor, so spin up a new thread
  std::cout << "[MAUROS2] - handle_accepted, spawn thread to: execute()" << std::endl;
  std::thread{execute, goal_handle}.detach();
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto ipc_setting = rclcpp::IntraProcessSetting::Disable;
  ipc_setting = rclcpp::IntraProcessSetting::Enable;

  if (ipc_setting == rclcpp::IntraProcessSetting::Enable) {
    std::cout << "[MAUROS2] - \n   INTRA-PROCESS ON   \n" << std::endl;
  } else {
    std::cout << "[MAUROS2] - \n   INTRA-PROCESS OFF  \n" << std::endl;
  }

  // Node options
  rclcpp::NodeOptions node_options = rclcpp::NodeOptions();
  node_options.use_intra_process_comms(true);
  node_options.start_parameter_services(false);
  node_options.start_parameter_event_publisher(false);

  ////////////////////////////// SERVER ///////////////////////////////////////
  auto server_options = rcl_action_server_get_default_options();

  server_options.status_topic_qos.depth = 2;
  server_options.goal_service_qos.depth = 2;
  server_options.result_service_qos.depth = 2;
  server_options.cancel_service_qos.depth = 2;
  server_options.feedback_topic_qos.depth = 2;

  server_options.status_topic_qos.durability = RMW_QOS_POLICY_DURABILITY_VOLATILE;
  server_options.goal_service_qos.durability = RMW_QOS_POLICY_DURABILITY_VOLATILE;
  server_options.result_service_qos.durability = RMW_QOS_POLICY_DURABILITY_VOLATILE;
  server_options.cancel_service_qos.durability = RMW_QOS_POLICY_DURABILITY_VOLATILE;
  server_options.feedback_topic_qos.durability = RMW_QOS_POLICY_DURABILITY_VOLATILE;

  // Goal handles that have results longer than this time are deallocated
  const std::chrono::milliseconds result_timeout{5000};
  server_options.result_timeout.nanoseconds = RCL_MS_TO_NS(result_timeout.count());

  auto server_node = rclcpp::Node::make_shared("minimal_action_server", node_options);

  auto action_server = rclcpp_action::create_server<Fibonacci>(
    server_node,
    "fibonacci",
    handle_goal,
    handle_cancel,
    handle_accepted,
    server_options,
    nullptr,
    ipc_setting);

  auto server_executor = std::make_shared<rclcpp::executors::EventsExecutor>();
  // auto server_executor = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();
  server_executor->add_node(server_node);

  std::thread service_spin_thread([=](){
      std::cout << "[MAUROS2] - server_executor->spin()" << std::endl;
      server_executor->spin();
  });
  service_spin_thread.detach();
  ////////////////////////////// END SERVER ////////////////////////////////


  ////////////////////////////// CLIENT ///////////////////////////////////////
  auto client_options = rcl_action_client_get_default_options();

  client_options.status_topic_qos.depth = 2;
  client_options.goal_service_qos.depth = 2;
  client_options.result_service_qos.depth = 2;
  client_options.cancel_service_qos.depth = 2;
  client_options.feedback_topic_qos.depth = 2;

  client_options.status_topic_qos.durability = RMW_QOS_POLICY_DURABILITY_VOLATILE;
  client_options.goal_service_qos.durability = RMW_QOS_POLICY_DURABILITY_VOLATILE;
  client_options.result_service_qos.durability = RMW_QOS_POLICY_DURABILITY_VOLATILE;
  client_options.cancel_service_qos.durability = RMW_QOS_POLICY_DURABILITY_VOLATILE;
  client_options.feedback_topic_qos.durability = RMW_QOS_POLICY_DURABILITY_VOLATILE;

  auto client_node = rclcpp::Node::make_shared("client", node_options);

  auto client_executor = std::make_shared<rclcpp::executors::EventsExecutor>();
  client_executor->add_node(client_node);

  auto action_client = rclcpp_action::create_client<Fibonacci>(
      client_node, "fibonacci", nullptr, client_options, ipc_setting);

  // Not supported on Stub DDS !!!

  // if (!action_client->wait_for_action_server(std::chrono::seconds(20))) {
  //   RCLCPP_ERROR(client_node->get_logger(), "Action server not available after waiting");
  //   return 1;
  // }

  // Ask server to achieve some goal and wait until it's accepted
  // Populate a goal
  auto goal_msg = Fibonacci::Goal();
  goal_msg.order = 4;

  RCLCPP_INFO(client_node->get_logger(), "Sending goal");

  auto send_goal_options = rclcpp_action::Client<Fibonacci>::SendGoalOptions();

  send_goal_options.result_callback =
    [](const typename ActionGoalHandle::WrappedResult & result)
    {std::cout << "[MAUROS2] - Client, call result_callback" << std::endl; (void)result;};

  send_goal_options.goal_response_callback =
    [](typename ActionGoalHandle::SharedPtr goal_handle)
    {std::cout << "[MAUROS2] - Client, call goal_response_callback"<< std::endl;(void)goal_handle;};

  send_goal_options.feedback_callback =
    [](typename ActionGoalHandle::SharedPtr handle,
      const std::shared_ptr<const Fibonacci::Feedback> feedback)
    {
      std::cout << "[MAUROS2] - Client, call feedback_callback"<< std::endl;(void)handle;
      auto & sequence = feedback->sequence;
      for (auto i : sequence) {
        std::cout << "[MAUROS2] - i: " << i << std::endl;
      }
    };

  auto goal_handle_future = action_client->async_send_goal(goal_msg, send_goal_options);

  if (client_executor->spin_until_future_complete(goal_handle_future) !=
    rclcpp::FutureReturnCode::SUCCESS)
  {
    RCLCPP_ERROR(client_node->get_logger(), "send goal call failed :(");
    return 1;
  }

  // Goal sent, future is complete.
  // Get result
  rclcpp_action::ClientGoalHandle<Fibonacci>::SharedPtr goal_handle = goal_handle_future.get();

  if (!goal_handle) {
    RCLCPP_ERROR(client_node->get_logger(), "Goal was rejected by server");
    return 1;
  }

  auto result_future = action_client->async_get_result(goal_handle);

  RCLCPP_INFO(client_node->get_logger(), "Waiting for result");

  // Wait result. To test cancel, set a low timeout (3 sec)
  auto wait_result = client_executor->spin_until_future_complete(
    result_future,
    std::chrono::seconds(10));

  // Expire goal
  // std::cout << "[MAUROS2] - Expire goal, sleep some time" << std::endl;
  // rclcpp::sleep_for(2 * result_timeout);

  // Test:
  // This should fail if goal exipred, goal_handle should not be valid
  // auto test = action_client->async_get_result(goal_handle);

  if (rclcpp::FutureReturnCode::TIMEOUT == wait_result)
  {
    // Cancel the goal since it is taking too long
    RCLCPP_INFO(client_node->get_logger(), "Canceling goal, taking too long!");
    auto cancel_result_future = action_client->async_cancel_goal(goal_handle);

    auto cancel_result = client_executor->spin_until_future_complete(cancel_result_future);

    if (cancel_result != rclcpp::FutureReturnCode::SUCCESS) {
      RCLCPP_ERROR(client_node->get_logger(), "Failed to cancel goal");
      rclcpp::shutdown();
      return 1;
    }

    // The cancel request has been accepted!
    // Get the cancel result
    RCLCPP_INFO(client_node->get_logger(), "Goal is being canceled, wait for result.");
    if (client_executor->spin_until_future_complete(result_future) !=
      rclcpp::FutureReturnCode::SUCCESS)
    {
      RCLCPP_ERROR(client_node->get_logger(), "Get result call failed :(");
      return 1;
    }

  }
  else if (rclcpp::FutureReturnCode::SUCCESS != wait_result)
  {
    RCLCPP_ERROR(client_node->get_logger(), "get cancel result call failed :(");
    rclcpp::shutdown();
    return 1;
  }

  // Expire goal
  // std::cout << "[MAUROS2] - Expire goal, sleep some time" << std::endl;
  // rclcpp::sleep_for(2 * result_timeout);

  rclcpp_action::ClientGoalHandle<Fibonacci>::WrappedResult wrapped_result = result_future.get();

  switch (wrapped_result.code) {
    case rclcpp_action::ResultCode::SUCCEEDED:
      break;
    case rclcpp_action::ResultCode::ABORTED:
      RCLCPP_ERROR(client_node->get_logger(), "Goal was aborted");
    case rclcpp_action::ResultCode::CANCELED:
      RCLCPP_ERROR(client_node->get_logger(), "Goal was canceled");
    default:
      RCLCPP_ERROR(client_node->get_logger(), "Unknown result code");
  }

  RCLCPP_INFO(client_node->get_logger(), "result received");
  for (auto number : wrapped_result.result->sequence) {
    RCLCPP_INFO(client_node->get_logger(), "%" PRId32, number);
  }

  rclcpp::shutdown();
  return 0;
}
