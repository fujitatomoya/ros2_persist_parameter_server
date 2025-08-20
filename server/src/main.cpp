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

#include <boost/program_options.hpp>

#include "parameter_server.h"

using namespace std;
using namespace boost::program_options;

int main(int argc, char **argv)
{
  int ret = EXIT_SUCCESS;
  // Force flush of the stdout buffer.
  setvbuf(stdout, NULL, _IONBF, BUFSIZ);

  // To call rclcpp::init_and_remove_ros_arguments at the beginning to prevent
  // from using new arguments "--ros-args" to support remapping node name
  // (such as "--ros-args --remap __node:=test1")
  // that boost program options failed to parse arguments
  auto nonros_args = rclcpp::init_and_remove_ros_arguments(argc, argv);

  options_description description("ROS2 parameter server command line interfaces");
  description.add_options()
    ("help,h", "help message to show interfaces")
    ("file-path,f", value<string>()->default_value("/tmp/parameter_server.yaml"),
    "volume path to load/store parameters in yaml format (default /tmp/parameter_server.yaml)")
    ("allow-declare,d", value<bool>()->default_value(true),
    "enable(true) / disable(false) allow_undeclared_parameters via node option (default true)")
    ("allow-override,o", value<bool>()->default_value(true),
    "enable(true) / disable(false) automatically_declare_parameters_from_overrides via node option (default true)")
    ("storing-period,s", value<unsigned int>()->default_value(60),
    "period in seconds for periodic persistent parameter storing (default 60). No periodic storing is performed if this parameter is set to 0")
    ("allow-dynamic-typing,t", value<bool>()->default_value(false),
    "When enabled (true), allows parameter type to change upon reading from persistence. Disabled (false) by default");

  variables_map vm;
  store(basic_command_line_parser<char>(nonros_args).options(description).run(), vm);
  notify(vm);

  std::string node_name = "parameter_server";
  string opt_file("/tmp/parameter_server.yaml");
  bool opt_allow_declare = true;
  bool opt_allow_override = true;
  unsigned int storing_period = 60;
  bool opt_allow_dynamic_typing = false;

  if (vm.count("help"))
  {
    cout << description << endl;
    rclcpp::shutdown();
    return ret;
  }
  else
  {
    opt_file = vm["file-path"].as<string>();
    opt_allow_declare = vm["allow-declare"].as<bool>();
    opt_allow_override = vm["allow-override"].as<bool>();
    storing_period = vm["storing-period"].as<unsigned int>();
    opt_allow_dynamic_typing = vm["allow-dynamic-typing"].as<bool>();
  }

  rclcpp::NodeOptions options = (
    rclcpp::NodeOptions()
    .allow_undeclared_parameters(opt_allow_declare)
    .automatically_declare_parameters_from_overrides(opt_allow_override)
    .append_parameter_override("allow_dynamic_typing", opt_allow_dynamic_typing)
    .append_parameter_override("storing_period", static_cast<int>(storing_period))
    );

  ParameterServer::SharedPtr node = nullptr;
  try
  {
    node = ParameterServer::make_shared(node_name, options, opt_file);
    if (node == nullptr)
    {
      throw std::bad_alloc();
    }

    RCLCPP_INFO(node->get_logger(),
      "Parameter Server node named: '%s' started and ready, and serving '%zu' parameters already!",
      node->get_fully_qualified_name(),
      node->list_parameters({}, rcl_interfaces::srv::ListParameters::Request::DEPTH_RECURSIVE).names.size());

    rclcpp::spin(node);
  }
  catch (const std::exception& e)
  {
    std::cerr << "Catch exception: " << e.what() << std::endl;
    ret = EXIT_FAILURE;
  }

  rclcpp::shutdown();
  return ret;
}
