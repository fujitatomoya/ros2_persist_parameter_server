// Copyright 2019 Sony Corporation
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

#ifndef __PARAMETER_SERVER_H__
#define __PARAMETER_SERVER_H__

#include <memory>
#include <string>
#include <sstream>
#include <atomic>

#include "rclcpp/rclcpp.hpp"
#include "std_srvs/srv/trigger.hpp"
#include "rclcpp_components/register_node_macro.hpp"
#include "yaml-cpp/yaml.h"

class ParameterServer : public rclcpp::Node
{
public:
  RCLCPP_SMART_PTR_DEFINITIONS(ParameterServer)

  ParameterServer(
    const std::string & node_name,
    const rclcpp::NodeOptions & options,
    const std::string & persistent_yaml_file);
  ~ParameterServer();

private:
  // Using custom yaml file same as yaml format of ros2 parameter as much as possible,
  // so use rcl_yaml_param_parser functions directly to load custom persistent yaml file.
  void LoadYamlFile();

  // To store yaml into a file, think it's more convenient to use yaml_cpp than libyaml.
  // (rcl_yaml_param_parser/libyaml not contain store function)
  void StoreYamlFile();

  // To check whether yaml file is valid
  void CheckYamlFile();
  void CheckYamlFile(const std::string& file);
  void ValidateYamlFile(YAML::Node node, const std::string& key = "");
  void SaveNode(YAML::Emitter& out, YAML::Node node, const std::string& key = "");

  // Check whether parameter name contains "persistent." in the parameter list
  bool CheckPersistentParam(const std::vector<rclcpp::Parameter> & parameters);

  // Check flag to store file
  std::atomic_bool param_update_;

  // yaml file to load/store
  std::string persistent_yaml_file_;

  // store changed(add, update) parameter name contains "persistent." after checking in 'parameter_events' callback
  std::set<std::string> changed_parameter_lists_;

  // To adapt the original yaml format that contain namespace(optional) and nodename(can be /**)
  bool parameter_use_stars_ = false;
  bool parameter_ns_exist_ = false;
  bool parameter_name_exist_ = false;
  std::string node_name_;

  // set parameters callback handler
  OnSetParametersCallbackHandle::SharedPtr callback_handler_;

  // for periodic storing to the file system
  rclcpp::TimerBase::SharedPtr timer_;

  bool allow_dynamic_typing_ = false;
  // For manual triggering of save
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr save_trigger_;
};

#endif // __PARAMETER_SERVER_H__
