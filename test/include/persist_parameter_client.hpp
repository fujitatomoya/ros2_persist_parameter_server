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

#ifndef __PARAMETER_CLIENT_H__
#define __PARAMETER_CLIENT_H__

#include "rclcpp/rclcpp.hpp"
#include "std_srvs/srv/trigger.hpp"

using namespace std::chrono_literals;

class PersistParametersClient : public rclcpp::Node
{
public:
  RCLCPP_SMART_PTR_DEFINITIONS(PersistParametersClient)

  RCLCPP_PUBLIC
  PersistParametersClient(
    const std::string & client_name,
    const rclcpp::NodeOptions & node_options = rclcpp::NodeOptions(),
    const std::string & remote_node_name = "parameter_server"
  );

  // Format the array value for easy output.
  template <typename ValueType>
  inline void format_array_output(std::ostringstream & ss, const std::vector<ValueType> & value_vec)
  {
    ss << "[ ";
    for(const auto & value : value_vec) {
      ss << value << " ";
    }
    ss << "]";

    return;
  }

  // Make sure the client and the server are connected through the Service.
  bool wait_param_server_ready()
  {
    bool ret = false;

    if(rclcpp::ok()) {
      RCLCPP_INFO(this->get_logger(), "Waiting 5 seconds to wait for the parameter server to be ready...");
      ret = sync_param_client_->wait_for_service(5s);
    }

    return ret;
  }

  /*
  * Read the parameter value specified by `param_name`.
  * @param param_name The name of parameter.
  * @param parameter The vector that holds the read result.
  * @return Operation as expected or not.
  */
  bool read_parameter(const std::string & param_name, std::vector<rclcpp::Parameter> & parameter);

  /*
  * Change the value of `param_name` to `param_value`.
  * The principle is to update if param exists, otherwise insert.
  *
  * @param param_name The name of parameter.
  * @param parameter_value The parameter value that you want to set.
  * @return Operation as expected or not.
  */
  template <typename ValueType>
  bool modify_parameter(const std::string & param_name, const ValueType & param_value)
  {
    bool ret = true;
    std::vector<rclcpp::Parameter> parameters;

    parameters.push_back(rclcpp::Parameter(param_name, rclcpp::ParameterValue(param_value)));
    auto set_param_result = sync_param_client_->set_parameters(parameters);
    for (auto & result : set_param_result)
    {
      if (!result.successful)
      {        
        RCLCPP_INFO(this->get_logger(), "SET OPERATION : Failed to set parameter: %s", result.reason.c_str());
        return false;
      }
    }
    RCLCPP_INFO(this->get_logger(), "SET OPERATION : Set parameter %s successfully.", param_name.c_str());

    return ret;
  }

  inline std::shared_ptr<std_srvs::srv::Trigger::Response> trigger_save() {
    auto trigger = std::make_shared<std_srvs::srv::Trigger::Request>();
    auto fut = this->save_trigger_client_->async_send_request(trigger);
    rclcpp::spin_until_future_complete(this->get_node_base_interface(), fut);
    return fut.get();
  }

  inline std::shared_ptr<std_srvs::srv::Trigger::Response> reload_yaml() {
    auto trigger = std::make_shared<std_srvs::srv::Trigger::Request>();
    auto fut = this->reload_trigger_client_->async_send_request(trigger);
    rclcpp::spin_until_future_complete(this->get_node_base_interface(), fut);
    return fut.get();
  }

private:
   std::unique_ptr<rclcpp::SyncParametersClient> sync_param_client_;
   std::shared_ptr<rclcpp::Client<std_srvs::srv::Trigger>> save_trigger_client_;
   std::shared_ptr<rclcpp::Client<std_srvs::srv::Trigger>> reload_trigger_client_;
};

#endif
