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

#ifndef PERSIST_PARAMETER_CLIENT_HPP_
#define PERSIST_PARAMETER_CLIENT_HPP_

#include <chrono>
#include <memory>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "std_srvs/srv/trigger.hpp"

/// Test helper node talking to the parameter server via ROS 2 parameter API and services.
class PersistParametersClient : public rclcpp::Node
{
public:
  RCLCPP_SMART_PTR_DEFINITIONS(PersistParametersClient)

  explicit PersistParametersClient(
    const std::string & client_name = "client",
    const std::string & remote_node_name = "parameter_server",
    std::chrono::milliseconds timeout = std::chrono::seconds(5));

  /// Wait until the parameter services of the remote parameter server are available.
  /**
   * \return true if the server is ready before the timeout expires, false otherwise.
   */
  bool wait_param_server_ready();

  /// Read the parameter specified by `param_name` from the parameter server.
  /**
   * \param param_name The name of the parameter.
   * \return The parameter. The type is PARAMETER_NOT_SET if the parameter does not exist
   *   or the request failed.
   */
  rclcpp::Parameter read_parameter(const std::string & param_name);

  /// Set the parameter `param_name` to `param_value` on the parameter server.
  /**
   * The parameter is implicitly declared on the server if it does not exist yet.
   *
   * \param param_name The name of the parameter.
   * \param param_value The value to set.
   * \return true if the parameter server accepted the change, false otherwise.
   */
  template<typename ValueType>
  bool modify_parameter(const std::string & param_name, const ValueType & param_value)
  {
    std::vector<rclcpp::Parameter> parameters;
    parameters.emplace_back(param_name, rclcpp::ParameterValue(param_value));
    auto results = sync_param_client_->set_parameters(parameters, timeout_);
    if (results.empty()) {
      RCLCPP_ERROR(
        this->get_logger(), "SET OPERATION : timed out setting parameter %s", param_name.c_str());
      return false;
    }
    for (const auto & result : results) {
      if (!result.successful) {
        RCLCPP_INFO(
          this->get_logger(), "SET OPERATION : failed to set parameter %s: %s",
          param_name.c_str(), result.reason.c_str());
        return false;
      }
    }
    RCLCPP_INFO(
      this->get_logger(), "SET OPERATION : set parameter %s successfully", param_name.c_str());
    return true;
  }

  /// Call the `~/save_params` service of the parameter server.
  bool trigger_save();

  /// Call the `~/reload_params` service of the parameter server.
  bool reload_params();

private:
  bool call_trigger(
    rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr client, const std::string & description);

  std::chrono::milliseconds timeout_;
  std::unique_ptr<rclcpp::SyncParametersClient> sync_param_client_;
  rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr save_client_;
  rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr reload_client_;
};

#endif  // PERSIST_PARAMETER_CLIENT_HPP_
