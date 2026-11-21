// Copyright 2020 Sony Corporation
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

#include "persist_parameter_client.hpp"

#include <memory>
#include <string>
#include <vector>

PersistParametersClient::PersistParametersClient(
  const std::string & client_name,
  const std::string & remote_node_name,
  std::chrono::milliseconds timeout)
: Node(client_name),
  timeout_(timeout)
{
  sync_param_client_ = std::make_unique<rclcpp::SyncParametersClient>(this, remote_node_name);
  save_client_ = create_client<std_srvs::srv::Trigger>(remote_node_name + "/save_params");
  reload_client_ = create_client<std_srvs::srv::Trigger>(remote_node_name + "/reload_params");
}

bool PersistParametersClient::wait_param_server_ready()
{
  if (!rclcpp::ok()) {
    return false;
  }
  RCLCPP_INFO(this->get_logger(), "Waiting for the parameter server to be ready...");
  return sync_param_client_->wait_for_service(timeout_) &&
         save_client_->wait_for_service(timeout_) &&
         reload_client_->wait_for_service(timeout_);
}

rclcpp::Parameter PersistParametersClient::read_parameter(const std::string & param_name)
{
  std::vector<rclcpp::Parameter> parameters =
    sync_param_client_->get_parameters({param_name}, timeout_);
  if (parameters.empty()) {
    RCLCPP_ERROR(
      this->get_logger(), "GET OPERATION : failed to get parameter %s", param_name.c_str());
    return rclcpp::Parameter(param_name);
  }
  const rclcpp::Parameter & param = parameters.front();
  if (param.get_type() == rclcpp::ParameterType::PARAMETER_NOT_SET) {
    RCLCPP_INFO(
      this->get_logger(), "GET OPERATION : parameter %s is not set", param_name.c_str());
  } else {
    RCLCPP_INFO(
      this->get_logger(), "GET OPERATION : parameter %s's value is %s",
      param_name.c_str(), param.value_to_string().c_str());
  }
  return param;
}

bool PersistParametersClient::trigger_save()
{
  return call_trigger(save_client_, "SAVE TRIGGER");
}

bool PersistParametersClient::reload_params()
{
  return call_trigger(reload_client_, "RELOAD TRIGGER");
}

bool PersistParametersClient::call_trigger(
  rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr client, const std::string & description)
{
  if (!client->wait_for_service(timeout_)) {
    RCLCPP_ERROR(this->get_logger(), "%s : service is not available", description.c_str());
    return false;
  }
  auto request = std::make_shared<std_srvs::srv::Trigger::Request>();
  auto future = client->async_send_request(request);
  auto rc = rclcpp::spin_until_future_complete(this->get_node_base_interface(), future, timeout_);
  if (rc != rclcpp::FutureReturnCode::SUCCESS) {
    RCLCPP_ERROR(
      this->get_logger(), "%s : spin_until_future_complete failed (%s)",
      description.c_str(), rclcpp::to_string(rc).c_str());
    return false;
  }
  auto response = future.get();
  if (!response->success) {
    RCLCPP_ERROR(
      this->get_logger(), "%s : request failed: %s",
      description.c_str(), response->message.c_str());
  }
  return response->success;
}
