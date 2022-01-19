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
  if (goal->order > 5) {
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
    std::cout << "Application call: goal_handle->succeed(result);" << std::endl;
    goal_handle->succeed(result);
    RCLCPP_INFO(rclcpp::get_logger("server"), "Goal Succeeded");
  }
}

void handle_accepted(const std::shared_ptr<GoalHandleFibonacci> goal_handle)
{
  // this needs to return quickly to avoid blocking the executor, so spin up a new thread
  std::cout << "handle_accepted, spawn thread to: execute()" << std::endl;
  std::thread{execute, goal_handle}.detach();
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto ipc_setting = rclcpp::IntraProcessSetting::Disable;
  ipc_setting = rclcpp::IntraProcessSetting::Enable;

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

  auto server_node = rclcpp::Node::make_shared("minimal_action_server");

  auto action_server = rclcpp_action::create_server<Fibonacci>(
    server_node,
    "fibonacci",
    handle_goal,
    handle_cancel,
    handle_accepted,
    server_options,
    nullptr,
    ipc_setting);

  auto server_executor = std::make_shared<rclcpp::executors::StaticSingleThreadedExecutor>();
  server_executor->add_node(server_node);

  std::thread service_spin_thread([=](){
      std::cout << "server_executor->spin()" << std::endl;
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

  auto client_node = rclcpp::Node::make_shared("client");

  auto action_client = rclcpp_action::create_client<Fibonacci>(
      client_node, "fibonacci", nullptr, client_options, ipc_setting);

  if (!action_client->wait_for_action_server(std::chrono::seconds(20))) {
    RCLCPP_ERROR(client_node->get_logger(), "Action server not available after waiting");
    return 1;
  }

  // Populate a goal
  auto goal_msg = Fibonacci::Goal();
  goal_msg.order = 4;

  RCLCPP_INFO(client_node->get_logger(), "Sending goal");
  // Ask server to achieve some goal and wait until it's accepted

  auto send_goal_options = rclcpp_action::Client<Fibonacci>::SendGoalOptions();

  send_goal_options.result_callback =
    [](const typename ActionGoalHandle::WrappedResult & result)
    {std::cout << "Calling result_callback" << std::endl; (void)result;};

  send_goal_options.goal_response_callback =
    [](typename ActionGoalHandle::SharedPtr goal_handle)
    {std::cout << "Calling goal_response_callback"<< std::endl;(void)goal_handle;};

  send_goal_options.feedback_callback =
    [](typename ActionGoalHandle::SharedPtr handle,
      const std::shared_ptr<const Fibonacci::Feedback> feedback)
    {std::cout << "Calling feedback_callback"<< std::endl;(void)handle;(void)feedback;};

  auto goal_handle_future = action_client->async_send_goal(goal_msg, send_goal_options);

  if (rclcpp::spin_until_future_complete(client_node, goal_handle_future) !=
    rclcpp::FutureReturnCode::SUCCESS)
  {
    RCLCPP_ERROR(client_node->get_logger(), "send goal call failed :(");
    return 1;
  }

  rclcpp_action::ClientGoalHandle<Fibonacci>::SharedPtr goal_handle = goal_handle_future.get();

  if (!goal_handle) {
    RCLCPP_ERROR(client_node->get_logger(), "Goal was rejected by server");
    return 1;
  }

  // Wait for the server to be done with the goal
  auto result_future = action_client->async_get_result(goal_handle);

  RCLCPP_INFO(client_node->get_logger(), "Waiting for result");

  if (rclcpp::spin_until_future_complete(client_node, result_future) !=
    rclcpp::FutureReturnCode::SUCCESS)
  {
    RCLCPP_ERROR(client_node->get_logger(), "get result call failed :(");
    return 1;
  }
  rclcpp_action::ClientGoalHandle<Fibonacci>::WrappedResult wrapped_result = result_future.get();

  switch (wrapped_result.code) {
    case rclcpp_action::ResultCode::SUCCEEDED:
      break;
    case rclcpp_action::ResultCode::ABORTED:
      RCLCPP_ERROR(client_node->get_logger(), "Goal was aborted");
      return 1;
    case rclcpp_action::ResultCode::CANCELED:
      RCLCPP_ERROR(client_node->get_logger(), "Goal was canceled");
      return 1;
    default:
      RCLCPP_ERROR(client_node->get_logger(), "Unknown result code");
      return 1;
  }

  RCLCPP_INFO(client_node->get_logger(), "result received");
  for (auto number : wrapped_result.result->sequence) {
    RCLCPP_INFO(client_node->get_logger(), "%" PRId32, number);
  }

  rclcpp::shutdown();
  return 0;
}
