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

#include <vector>
#include <sstream>
#include <functional>

#include "persist_parameter_client.hpp"

PersistParametersClient::PersistParametersClient(
  const std::string & client_name,
  const rclcpp::NodeOptions & node_options,
  const std::string & remote_node_name)
  : Node(client_name, node_options)
{
  sync_param_client_ = std::make_unique<rclcpp::SyncParametersClient>(this, remote_node_name);
  save_trigger_client_ = create_client<std_srvs::srv::Trigger>(remote_node_name + "/save_params");
  reload_trigger_client_ = create_client<std_srvs::srv::Trigger>(remote_node_name + "/reload_params");
}

bool PersistParametersClient::read_parameter(const std::string & param_name, std::vector<rclcpp::Parameter> & parameter)
{
  bool ret = true;

  parameter = sync_param_client_->get_parameters({param_name});
  for(auto & param : parameter)
  {
      switch (param.get_type())
      {
        case rclcpp::ParameterType::PARAMETER_NOT_SET:
        {
          RCLCPP_INFO(this->get_logger(), "READ OPERATION : parameter %s was not set(or deleted), it will not be stored", param_name.c_str());
          break;
        }
        case rclcpp::ParameterType::PARAMETER_BOOL:
        {
          bool value = param.as_bool();
          RCLCPP_INFO(this->get_logger(), "GET OPERATION : parameter %s's value is %s", param_name.c_str(), value?"true":"false");
          break;
        }
        case rclcpp::ParameterType::PARAMETER_INTEGER:
        {
          int64_t value = param.as_int();
          RCLCPP_INFO(this->get_logger(), "GET OPERATION : parameter %s's value is %ld", param_name.c_str(), value);
          break;
        }
        case rclcpp::ParameterType::PARAMETER_DOUBLE:
        {
          double value = param.as_double();
          RCLCPP_INFO(this->get_logger(), "GET OPERATION : parameter %s's value is %lf", param_name.c_str(), value);
          break;
        }
        case rclcpp::ParameterType::PARAMETER_STRING:
        {
          std::string value = param.as_string();
          RCLCPP_INFO(this->get_logger(), "GET OPERATION : parameter %s's value is %s", param_name.c_str(), value.c_str());
          break;
        }
        case rclcpp::ParameterType::PARAMETER_BYTE_ARRAY:
        {
          std::ostringstream ss;
          auto array = param.as_byte_array();
          format_array_output<uint8_t>(ss, array);
          RCLCPP_INFO(this->get_logger(), "GET OPERATION : parameter %s's value is %s", param_name.c_str(), ss.str().c_str());
          break;
        }
        case rclcpp::ParameterType::PARAMETER_BOOL_ARRAY:
        {
          std::ostringstream ss;
          auto array = param.as_bool_array();
          format_array_output<bool>(ss, array);
          RCLCPP_INFO(this->get_logger(), "GET OPERATION : parameter %s's value is %s", param_name.c_str(), ss.str().c_str());
          break;
        }
        case rclcpp::ParameterType::PARAMETER_INTEGER_ARRAY:
        {
          std::ostringstream ss;
          auto array = param.as_integer_array();
          format_array_output<int64_t>(ss, array);
          RCLCPP_INFO(this->get_logger(), "GET OPERATION : parameter %s's value is %s", param_name.c_str(), ss.str().c_str());   
          break;
        }
        case rclcpp::ParameterType::PARAMETER_DOUBLE_ARRAY:
        {
          std::ostringstream ss;
          auto array = param.as_double_array();
          format_array_output<double>(ss, array);
          RCLCPP_INFO(this->get_logger(), "GET OPERATION : parameter %s's value is %s", param_name.c_str(), ss.str().c_str());
          break;
        }
        case rclcpp::ParameterType::PARAMETER_STRING_ARRAY:
        {
          std::ostringstream ss;
          auto array = param.as_string_array();
          format_array_output<std::string>(ss, array);
          RCLCPP_INFO(this->get_logger(), "GET OPERATION : parameter %s's value is %s", param_name.c_str(), ss.str().c_str());
          break;
        }
        default: {
          ret = false;
          RCLCPP_INFO(this->get_logger(), "parameter %s unsupported type %d", param_name.c_str(), param.get_type());
          break;
      }
    }
  }

  return ret;
}
