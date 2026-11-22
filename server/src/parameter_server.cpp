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

#include "parameter_server.hpp"

#include <chrono>
#include <cinttypes>
#include <fstream>
#include <limits>
#include <map>
#include <boost/algorithm/string.hpp>
#include <boost/filesystem.hpp>

#include "rcl_yaml_param_parser/parser.h"
#include "rclcpp/parameter.hpp"
#include "rclcpp/parameter_map.hpp"
#include "rclcpp/version.h"

#define ROS_PARAMETER_KEY     "ros__parameters"
#define ROS_PARAMETER_DOT_KEY "ros__parameters."
#define PERSISTENT_KEY        "persistent"
#define PERSISTENT_DOT_KEY    "persistent."

// Server configuration parameter names
#define STORING_PERIOD_PARAM        "storing_period"
#define ALLOW_DYNAMIC_TYPING_PARAM  "allow_dynamic_typing"
#define MUST_SAVE_ON_UPDATE_PARAM   "must_save_on_update"

// Upper bound of storing_period in seconds, so that the wall timer period
// (converted to nanoseconds) can never overflow.
static constexpr int64_t MAX_STORING_PERIOD_SEC =
  std::numeric_limits<int64_t>::max() / 1000000000LL;

#if defined(_MSC_VER)
#define PARAMETER_SERVER_FUNCTION __FUNCSIG__
#else
#define PARAMETER_SERVER_FUNCTION __PRETTY_FUNCTION__
#endif

/**
 * @brief Converts a double to string with proper floating-point representation.
 *
 * Ensures the output contains either a decimal point with .0 suffix (e.g., 10.0)
 * or scientific notation (e.g., 1e5), never plain integers.
 * @param v The double value to convert.
 * @param precision The number of digits to represent. Defaults to max_digits10 for full precision.
 * @return std::string The string representation of the double.
 */
static std::string convertDoubleToString(
  double v,
  const size_t precision = std::numeric_limits<double>::max_digits10)
{
  // Convert to string using stringstream with classic locale
  // for consistent decimal point formatting
  auto ss = std::stringstream{};
  ss.imbue(std::locale::classic());
  ss << std::setprecision(precision) << v;
  auto str = ss.str();

  // Append .0 if the string lacks both decimal point and scientific notation
  if (str.find('.') == std::string::npos && str.find('e') == std::string::npos) {
    str += ".0";
  }
  return str;
}

ParameterServer::ParameterServer(
  const std::string & node_name,
  const rclcpp::NodeOptions & options,
  const std::string & persistent_yaml_file)
: Node(node_name, options),
  param_update_(false),
  persistent_yaml_file_(persistent_yaml_file),
  node_name_(get_name())
{
  RCLCPP_DEBUG(
    this->get_logger(), "%s yaml:%s", PARAMETER_SERVER_FUNCTION, persistent_yaml_file_.c_str());

#if RCLCPP_VERSION_MAJOR < 17
  server_param_subscriber_ = std::make_shared<rclcpp::ParameterEventHandler>(this);
#endif

  // Declare the server configuration parameters as node parameters.
  // The values passed via command line are applied as parameter overrides,
  // so they are picked up here regardless of automatically_declare_parameters_from_overrides.
  DeclareServerParameters();

  if (allow_dynamic_typing_) {
    RCLCPP_INFO(
      this->get_logger(),
      "Dynamic typing enabled. Read persistent parameters will be dynamically typed.");
  }

  if (must_save_on_update_) {
    RCLCPP_INFO(
      this->get_logger(),
      "Save on update enabled. Parameters will be saved when set.");
  }

  if (storing_period_ < 0 || storing_period_ > MAX_STORING_PERIOD_SEC) {
    RCLCPP_WARN(
      this->get_logger(),
      "storing_period parameter value (%" PRId64 ") is not valid, treating as 0", storing_period_);
    storing_period_ = 0;
    // keep the declared parameter value consistent with the actual behavior
    this->set_parameter(rclcpp::Parameter(STORING_PERIOD_PARAM, storing_period_));
  }

  if (!storing_period_) {
    RCLCPP_INFO(
      this->get_logger(), "Period is 0. Will not perform periodic persistent parameter storing");
  } else {
    timer_ = this->create_wall_timer(
      std::chrono::seconds(storing_period_),
      std::bind(&ParameterServer::TimerCallback, this)
    );

    RCLCPP_INFO(
      this->get_logger(), "Will perform periodic persistent parameter storing every %" PRId64 "s",
      storing_period_);
  }

#if RCLCPP_VERSION_MAJOR < 17
  auto allow_dynamic_typing_param_change_callback =
    [this](const rclcpp::Parameter & p) {
      updateDynamicTyping(p);
    };
  dynamic_typing_callback_handle_ = server_param_subscriber_->add_parameter_callback(
    ALLOW_DYNAMIC_TYPING_PARAM,
    allow_dynamic_typing_param_change_callback
  );

  auto storing_period_param_change_callback =
    [this](const rclcpp::Parameter & p) {
      updateStoringTimer(p);
    };

  storing_period_callback_handle_ = server_param_subscriber_->add_parameter_callback(
    STORING_PERIOD_PARAM,
    storing_period_param_change_callback
  );

  // For ROS Humble (rclcpp 16.x) compatibility, handle save-on-update with a parameter
  // event callback, since parameter events are published after the parameters are
  // actually applied to the node. Post-set callbacks were added in Iron (rclcpp 17.x).
  auto param_event_callback =
    [this](const rcl_interfaces::msg::ParameterEvent & event) {
      if (!must_save_on_update_ ||
        event.node != std::string(this->get_fully_qualified_name()))
      {
        return;
      }
      std::vector<rclcpp::Parameter> params;
      for (const rcl_interfaces::msg::Parameter & p : event.new_parameters) {
        params.push_back(rclcpp::Parameter::from_parameter_msg(p));
      }
      for (const rcl_interfaces::msg::Parameter & p : event.changed_parameters) {
        params.push_back(rclcpp::Parameter::from_parameter_msg(p));
      }
      if (CheckPersistentParam(params)) {
        this->StoreYamlFile();
      }
    };
  param_event_callback_handle_ = server_param_subscriber_->add_parameter_event_callback(
    param_event_callback
  );
#endif

  // Declare a parameter change request callback
  auto param_change_callback =
    [this](const std::vector<rclcpp::Parameter> & parameters)
    {
      // Reject the whole set operation before any side effects are applied
      auto result = checkValidParams(parameters);
      if (!result.successful) {
        return result;
      }

#if RCLCPP_VERSION_MAJOR < 17
      for (const rclcpp::Parameter & param : parameters) {
        if(param.get_name() == MUST_SAVE_ON_UPDATE_PARAM) {
          RCLCPP_INFO(
              this->get_logger(), "Save on update parameter value changed to %s",
              param.as_bool() ? "true" : "false");
          must_save_on_update_ = param.as_bool();
        }
      }
#endif

      if (CheckPersistentParam(parameters)) {
        if (!param_update_) {
          param_update_ = true;
        }
<<<<<<< HEAD
=======

#if RCLCPP_VERSION_MAJOR < 17
        // For ROS Humble (rclcpp 16.x) compatibility, handle save-on-update in on-set callback
        // Post-set callbacks were added in Iron (rclcpp 17.x) and later
        if(must_save_on_update_) {
          this->StoreYamlFile();
        }
#endif
>>>>>>> 1caf6b5 (Integrate unit and system tests via ament_add_gtest (#18) (#148))
      }

      return result;
    };

  // callback_handler_ needs to be alive to keep the callback functional
  callback_handler_ = this->add_on_set_parameters_callback(param_change_callback);

#if RCLCPP_VERSION_MAJOR >= 17
  // Use post-set callback for Iron (rclcpp 17.x) and later distributions
  auto post_param_change_callback =
    [this](const std::vector<rclcpp::Parameter> & parameters)
    {
      // Update behavior based on the updated parameter
      for (const rclcpp::Parameter & param : parameters) {
        if(param.get_name() == MUST_SAVE_ON_UPDATE_PARAM) {
          RCLCPP_INFO(
            this->get_logger(), "Save on update parameter value changed to %s",
            param.as_bool() ? "true" : "false");
          must_save_on_update_ = param.as_bool();
        } else if(param.get_name() == STORING_PERIOD_PARAM) {
          updateStoringTimer(param);
        } else if(param.get_name() == ALLOW_DYNAMIC_TYPING_PARAM) {
          updateDynamicTyping(param);
        }
      }

      if (CheckPersistentParam(parameters)) {
        if(must_save_on_update_) {
          this->StoreYamlFile();
        }
      }
    };
  post_set_callback_handler_ = this->add_post_set_parameters_callback(post_param_change_callback);
#endif

  save_trigger_ = this->create_service<std_srvs::srv::Trigger>("~/save_params",
      [this]([[maybe_unused]] const std_srvs::srv::Trigger::Request::SharedPtr & req,
      [[maybe_unused]] const std_srvs::srv::Trigger::Response::SharedPtr & res
      ) {
        RCLCPP_INFO(this->get_logger(), "Parameter save manually requested");
        try {
          this->StoreYamlFile();
          res->success = true;
          res->message = "Parameters saved successfully";
        } catch(const std::exception & ex) {
          std::ostringstream ss;
          ss << "Parameters could not be saved. Error: " << ex.what();
          res->success = false;
          res->message = ss.str();
        }
  });

  reload_trigger_ = this->create_service<std_srvs::srv::Trigger>("~/reload_params",
      [this]([[maybe_unused]] const std_srvs::srv::Trigger::Request::SharedPtr & req,
      [[maybe_unused]] const std_srvs::srv::Trigger::Response::SharedPtr & res
      ) {
        RCLCPP_INFO(this->get_logger(), "Parameter reload manually requested");
        try {
          this->LoadYamlFile();
          res->success = true;
          res->message = "Parameters reloaded";
        } catch(const std::exception & ex) {
          std::ostringstream ss;
          ss << "Parameters could not be reloaded. Error: " << ex.what();
          res->success = false;
          res->message = ss.str();
        }
  });

  LoadYamlFile();
}

ParameterServer::~ParameterServer()
{
  RCLCPP_DEBUG(this->get_logger(), "%s", PARAMETER_SERVER_FUNCTION);
  this->remove_on_set_parameters_callback(callback_handler_.get());
#if RCLCPP_VERSION_MAJOR >= 17
  if (post_set_callback_handler_) {
    this->remove_post_set_parameters_callback(post_set_callback_handler_.get());
  }
#endif
  StoreYamlFile();
}

void ParameterServer::TimerCallback()
{
  StoreYamlFile();
}

void ParameterServer::DeclareServerParameters()
{
  RCLCPP_DEBUG(this->get_logger(), "%s", PARAMETER_SERVER_FUNCTION);

  {
    rcl_interfaces::msg::ParameterDescriptor descriptor;
    descriptor.name = STORING_PERIOD_PARAM;
    descriptor.description =
      "Period in seconds for periodic persistent parameter storing. "
      "0 disables the periodic storing. Can be changed at runtime.";
    storing_period_ = DeclareOrGetServerParameter<int64_t>(
      STORING_PERIOD_PARAM, storing_period_, descriptor);
  }

  {
    rcl_interfaces::msg::ParameterDescriptor descriptor;
    descriptor.name = ALLOW_DYNAMIC_TYPING_PARAM;
    descriptor.description =
      "Allow the type of the parameters served by the parameter server to be changed "
      "after declaration. Can be changed at runtime.";
    allow_dynamic_typing_ = DeclareOrGetServerParameter<bool>(
      ALLOW_DYNAMIC_TYPING_PARAM, allow_dynamic_typing_, descriptor);
  }

  {
    rcl_interfaces::msg::ParameterDescriptor descriptor;
    descriptor.name = MUST_SAVE_ON_UPDATE_PARAM;
    descriptor.description =
      "Store the persistent parameters to the file system each time one of them is changed. "
      "Can be changed at runtime.";
    must_save_on_update_ = DeclareOrGetServerParameter<bool>(
      MUST_SAVE_ON_UPDATE_PARAM, must_save_on_update_, descriptor);
  }
}

rcl_interfaces::msg::SetParametersResult ParameterServer::checkValidParams(
  const std::vector<rclcpp::Parameter> & parameters)
{
  auto result = rcl_interfaces::msg::SetParametersResult();
  result.successful = true;

  for (const rclcpp::Parameter & param : parameters) {
    const std::string & name = param.get_name();
    const rclcpp::ParameterType type = param.get_type();

    if (name == STORING_PERIOD_PARAM) {
      if (type != rclcpp::ParameterType::PARAMETER_INTEGER) {
        result.successful = false;
        result.reason = std::string(STORING_PERIOD_PARAM) +
          " must be an integer value in seconds, and cannot be deleted.";
        return result;
      }
      if (param.as_int() < 0) {
        result.successful = false;
        result.reason = "Storing period cannot be a negative value.";
        return result;
      }
      if (param.as_int() > MAX_STORING_PERIOD_SEC) {
        result.successful = false;
        result.reason = "Storing period cannot be greater than " +
          std::to_string(MAX_STORING_PERIOD_SEC) + " seconds.";
        return result;
      }
    } else if (name == ALLOW_DYNAMIC_TYPING_PARAM || name == MUST_SAVE_ON_UPDATE_PARAM) {
      if (type != rclcpp::ParameterType::PARAMETER_BOOL) {
        result.successful = false;
        result.reason = name + " must be a boolean value, and cannot be deleted.";
        return result;
      }
    } else if (!allow_dynamic_typing_ && this->has_parameter(name)) {
      // Enforce the typing policy here instead of relying on the rclcpp descriptor,
      // so that allow_dynamic_typing can be reconfigured at runtime for the
      // parameters that are already declared.
      const rclcpp::ParameterType current_type = this->get_parameter(name).get_type();
      if (current_type != rclcpp::ParameterType::PARAMETER_NOT_SET && current_type != type) {
        result.successful = false;
        result.reason = "Wrong parameter type, parameter {" + name + "} is of type {" +
          rclcpp::to_string(current_type) + "}, setting it to {" + rclcpp::to_string(type) +
          "} is not allowed since " + ALLOW_DYNAMIC_TYPING_PARAM + " is false.";
        return result;
      }
    }
  }

  return result;
}

void ParameterServer::updateStoringTimer(const rclcpp::Parameter & param)
{
  const int64_t new_storing_period = param.as_int();

  RCLCPP_INFO(
    this->get_logger(), "Storing period param value changed to %" PRId64,
    new_storing_period);

  if (timer_) {
    timer_->cancel();
    timer_.reset();
  }

  storing_period_ = new_storing_period;
  if (storing_period_ > 0) {
    timer_ = this->create_wall_timer(
      std::chrono::seconds(storing_period_),
      std::bind(&ParameterServer::TimerCallback, this)
    );
  }
}

void ParameterServer::updateDynamicTyping(const rclcpp::Parameter & param)
{
  RCLCPP_INFO(
    this->get_logger(), "Allow dynamic typing param value changed to %s",
    param.as_bool() ? "true" : "false");
  allow_dynamic_typing_ = param.as_bool();
}

// Add a limitation that A node that is a map in custom YAML file can't contain '.' in the key name
void ParameterServer::ValidateYamlFile(YAML::Node node, const std::string & key)
{
  for (YAML::const_iterator it = node.begin(); it != node.end(); ++it) {
    if (it->second.Type() == YAML::NodeType::Map) {
      std::string key_name = key;
      std::string tag = it->first.as<std::string>();
      key_name += "[" + tag + "]";
      if (tag.find(".") != std::string::npos) {
        std::ostringstream ss;
        ss << "Custom YAML file '" << persistent_yaml_file_ << " format is invalid. "
           << "[A node(" << key_name <<
          ") that is a map in custom YAML file can't contain '.' in the key name";
        throw std::runtime_error(ss.str());
      }

      ValidateYamlFile(it->second, key_name);
    }
  }
}

void ParameterServer::CheckYamlFile()
{
  CheckYamlFile(persistent_yaml_file_);
}

void ParameterServer::CheckYamlFile(const std::string & file)
{
  RCLCPP_DEBUG(this->get_logger(), "%s", PARAMETER_SERVER_FUNCTION);
  YAML::Node parameter_config = YAML::LoadFile(file);
  // check format "YAML must be dictionary type and level 1 can only have one key"
  if ((parameter_config.size() == 1 && parameter_config.Type() != YAML::NodeType::Map) ||
    parameter_config.size() > 1)
  {
    std::ostringstream ss;
    ss << "Custom YAML file '" << file <<
      " format is invalid. [YAML must be dictionary type and level 1 can only have one key]";
    throw std::runtime_error(ss.str());
  }

  if (parameter_config.size() == 1 && parameter_config.Type() == YAML::NodeType::Map) {
    if (parameter_config["/**"]) {
      parameter_use_stars_ = true;
    } else {
      if (parameter_config[node_name_] && parameter_config[node_name_]["ros__parameters"]) {
        parameter_name_exist_ = true;
      } else {
        std::string tmp = "/" + node_name_;
        if (parameter_config[tmp] && parameter_config[tmp]["ros__parameters"]) {
          parameter_name_exist_ = true;
          node_name_ = tmp;
        }
      }

      if (!parameter_name_exist_) {
        if (parameter_config[get_namespace()] &&
          parameter_config[get_namespace()][node_name_] &&
          parameter_config[get_namespace()][node_name_]["ros__parameters"])
        {
          parameter_ns_exist_ = true;
          parameter_name_exist_ = true;
        }
      }
    }

    if (!parameter_use_stars_ && !parameter_ns_exist_ && !parameter_name_exist_) {
      std::ostringstream ss;
      ss << "Custom YAML file '" << file
         << " content is invalid. [namespace can be optional or '" << get_namespace() <<
        "', but node name must be exist with a concrete name'"
         << get_name() << "' or '/**']";
      throw std::runtime_error(ss.str());
    }
  }

  ValidateYamlFile(parameter_config);
}

void ParameterServer::LoadYamlFile()
{
  RCLCPP_DEBUG(this->get_logger(), "%s", PARAMETER_SERVER_FUNCTION);
  // check whether yaml file exist
  if (!boost::filesystem::exists(persistent_yaml_file_)) {
    RCLCPP_WARN(
      this->get_logger(),
      "Custom YAML file %s not exist", persistent_yaml_file_.c_str());
    return;
  }

  // check whether yaml file is valid
  CheckYamlFile();

  // use rcl_yaml_param_parser to load custom yaml file
  rclcpp::node_interfaces::NodeBaseInterface::SharedPtr node_base = this->get_node_base_interface();
  rclcpp::node_interfaces::NodeParametersInterface::SharedPtr node_parameters =
    get_node_parameters_interface();

  // Get the node options
  const rcl_node_t * node = node_base->get_rcl_node_handle();
  if (nullptr == node) {
    throw std::runtime_error("Need valid node handle in NodeParameters");
  }
  const rcl_node_options_t * options = rcl_node_get_options(node);
  if (nullptr == options) {
    throw std::runtime_error("Need valid node options in NodeParameters");
  }

  rcl_params_t * yaml_params = rcl_yaml_node_struct_init(options->allocator);
  if (nullptr == yaml_params) {
    throw std::bad_alloc();
  }
  if (!rcl_parse_yaml_file(persistent_yaml_file_.c_str(), yaml_params)) {
    std::ostringstream ss;
    ss << "Failed to parse parameters from custom yaml file '" << persistent_yaml_file_ << "': " <<
      rcl_get_error_string().str;
    rcl_reset_error();
    throw std::runtime_error(ss.str());
  }

  rclcpp::ParameterMap initial_map = rclcpp::parameter_map_from(yaml_params);
  rcl_yaml_node_struct_fini(yaml_params);

  for (auto iter = initial_map.begin(); initial_map.end() != iter; iter++) {
    if (iter->first == "/**" || iter->first == node_base->get_fully_qualified_name()) {
      // set (add, update) parameter with custom yaml file
      for (auto & param : iter->second) {
        std::string name = param.get_name();
        rclcpp::ParameterValue value = rclcpp::ParameterValue(param.get_value_message());

        bool has_parameter_flag = node_parameters->has_parameter(name);
        bool undeclare_exist_override = false;

        if (!has_parameter_flag) {
          // declare parameter
          RCLCPP_DEBUG(this->get_logger(), "declare %s %s", name.c_str(), to_string(value).c_str());
          // Always declare as dynamically typed in rclcpp. The typing policy is enforced
          // by the server itself (see checkValidParams) based on allow_dynamic_typing,
          // so that it can be reconfigured at runtime for already declared parameters.
          rcl_interfaces::msg::ParameterDescriptor descriptor;
          descriptor.dynamic_typing = true;
          node_parameters->declare_parameter(name, value, descriptor);

          // 1. if automatically_declare_parameters_from_overrides is false,
          // parameter from __params: not declared but saved in overrides list,
          // to check "has_parameter" return false and
          // to call "declare_parameter" will use value of overrides list to replace passed value.
          // 2. custom yaml file need to override whatever
          // the automatically_declare_parameters_from_overrides is,
          // continue to check if name exist in overrides, if yes, set_parameters later
          const std::map<std::string,
            rclcpp::ParameterValue> & om = node_parameters->get_parameter_overrides();
          if (om.find(name) != om.end()) {
            undeclare_exist_override = true;
          }
        }

        if (has_parameter_flag || undeclare_exist_override) {
          RCLCPP_DEBUG(this->get_logger(),
            "set %s %s (has_parameter_flag:%d, undeclare_exist_override:%d",
            name.c_str(), to_string(value).c_str(),
            has_parameter_flag, undeclare_exist_override);

          auto set_parameters_results = node_parameters->set_parameters({
            rclcpp::Parameter(name, value)
          });
          for (auto & result : set_parameters_results) {
            if (!result.successful) {
              RCLCPP_WARN(
                get_logger(),
                "Failed to set parameter: %s", result.reason.c_str());
            }
          }
        }
      }
    }
  }
}

template<typename T, typename Iter>
void setConfigParam(YAML::Node node, Iter begin, Iter end, T value)
{
  if (begin == end) {
    return;
  }
  auto tag = *begin;

  if (std::next(begin) == end) {
    if constexpr (std::is_same_v<T, double>) {
      node[tag] = convertDoubleToString(value);
    } else {
      node[tag] = value;
    }
    return;
  }
  if (!node[tag]) {
    node[tag] = YAML::Node(YAML::NodeType::Map);
  } else {
    // if tag is not map, set all left tags as a key with the value
    if (node[tag].Type() != YAML::NodeType::Map) {
      while (true) {
        ++begin;
        tag += "." + *begin;

        if (std::next(begin) == end) {
          if constexpr (std::is_same_v<T, double>) {
            node[tag] = convertDoubleToString(value);
          } else {
            node[tag] = value;
          }
          return;
        }
      }
    }
  }

  setConfigParam(node[tag], std::next(begin), end, value);
}

template<typename T>
void updateConfigParam(YAML::Node node, const std::vector<std::string> & key_name_list, T value)
{
  setConfigParam(node, key_name_list.begin(), key_name_list.end(), value);
}

void ParameterServer::SaveNode(YAML::Emitter & out, YAML::Node node, const std::string & key)
{
  out << YAML::BeginMap;
  for (YAML::const_iterator it = node.begin(); it != node.end(); ++it) {
    out << YAML::Key << it->first;

    std::string key_name;
    if (key.empty()) {
      key_name = it->first.as<std::string>();
    } else {
      key_name = key + "." + it->first.as<std::string>();
    }

    if (it->second.Type() == YAML::NodeType::Map) {
      SaveNode(out, it->second, key_name);
    } else {
      std::string ros_parameter_key = ROS_PARAMETER_DOT_KEY;
      std::size_t pos = key_name.find(ros_parameter_key + PERSISTENT_DOT_KEY);
      if (pos == std::string::npos) {
        // not a persistent key
        out << YAML::Value << it->second;
      } else {
        std::size_t pos = key_name.find(ros_parameter_key);
        std::string name = key_name.substr(pos + ros_parameter_key.length());
        // get key type of parameter
        rclcpp::Parameter parameter = get_parameter(name);
        switch (parameter.get_type()) {
          case rclcpp::ParameterType::PARAMETER_NOT_SET:
            {
              RCLCPP_INFO(this->get_logger(),
              "parameter %s not set(or deleted), it will not be stored", name.c_str());
              break;
            }
          case rclcpp::ParameterType::PARAMETER_BOOL:
            {
              bool value = parameter.as_bool();
              out << YAML::Value << value;
              break;
            }
          case rclcpp::ParameterType::PARAMETER_INTEGER:
            {
              int64_t value = parameter.as_int();
              out << YAML::Value << value;
              break;
            }
          case rclcpp::ParameterType::PARAMETER_DOUBLE:
            {
              double value = parameter.as_double();
              out << YAML::Value << convertDoubleToString(value);
              break;
            }
          case rclcpp::ParameterType::PARAMETER_STRING:
            {
              std::string value = parameter.as_string();
              out << YAML::Value << YAML::SingleQuoted << value;
              break;
            }
          case rclcpp::ParameterType::PARAMETER_BYTE_ARRAY:
            {
              // TODO(fujitatomoya): rcl_yaml_param_parser does not support byte array,
              // use int array temporarily
              auto array = parameter.as_byte_array();
              std::vector<int64_t> int_array;
              for (uint8_t b : array) {
                int_array.push_back(int64_t(b));
              }
              out << YAML::Value << YAML::Flow << int_array;
              break;
            }
          case rclcpp::ParameterType::PARAMETER_BOOL_ARRAY:
            {
              auto array = parameter.as_bool_array();
              out << YAML::Value << YAML::Flow << array;
              break;
            }
          case rclcpp::ParameterType::PARAMETER_INTEGER_ARRAY:
            {
              auto array = parameter.as_integer_array();
              out << YAML::Value << YAML::Flow << array;
              break;
            }
          case rclcpp::ParameterType::PARAMETER_DOUBLE_ARRAY:
            {
              auto array = parameter.as_double_array();
              std::vector<std::string> str_array;
              for (auto v : array) {
                str_array.push_back(convertDoubleToString(v));
              }
              out << YAML::Value << YAML::Flow << str_array;
              break;
            }
          case rclcpp::ParameterType::PARAMETER_STRING_ARRAY:
            {
              auto array = parameter.as_string_array();
              out << YAML::Value << YAML::Flow << YAML::SingleQuoted << array;
              break;
            }
          default: {
              RCLCPP_WARN(
              this->get_logger(),
              "parameter %s unsupported type %d",
              name.c_str(), parameter.get_type());
              break;
            }
        }
      }
    }
  }
  out << YAML::EndMap;
}

void ParameterServer::StoreYamlFile()
{
  RCLCPP_DEBUG(this->get_logger(), "%s", PARAMETER_SERVER_FUNCTION);

  if (param_update_) {
    // Store yaml at finalization
    YAML::Node parameter_config;
    if (boost::filesystem::exists(persistent_yaml_file_)) {
      parameter_config = YAML::LoadFile(persistent_yaml_file_);
    }

    // if file is empty or bad format, reset it to map
    if (parameter_config.Type() != YAML::NodeType::Map) {
      parameter_config = YAML::Node(YAML::NodeType::Map);
    }

    YAML::Node node_parameter;

    if (parameter_use_stars_) {
      node_parameter = parameter_config["/**"][ROS_PARAMETER_KEY];
    } else if (parameter_ns_exist_ && parameter_name_exist_) {
      node_parameter = parameter_config[get_namespace()][node_name_][ROS_PARAMETER_KEY];
    } else if (parameter_name_exist_) {
      node_parameter = parameter_config[node_name_][ROS_PARAMETER_KEY];
    } else {
      node_parameter = parameter_config[get_namespace()][node_name_][ROS_PARAMETER_KEY];
    }

    node_parameter[PERSISTENT_KEY] = YAML::Node(YAML::NodeType::Map);

    for (std::set<std::string>::iterator iter = changed_parameter_lists_.begin();
      iter != changed_parameter_lists_.end();
      ++iter)
    {
      std::string name = *iter;
      // split parameter name
      std::vector<std::string> key_name_list;
      boost::split(key_name_list, name, boost::is_any_of("."));

      rclcpp::Parameter parameter = get_parameter(name);

      switch (parameter.get_type()) {
        case rclcpp::ParameterType::PARAMETER_NOT_SET:
          {
            RCLCPP_INFO(this->get_logger(), "parameter %s is not set, it will not be stored",
            name.c_str());
            break;
          }
        case rclcpp::ParameterType::PARAMETER_BOOL:
          {
            bool value = parameter.as_bool();
            updateConfigParam(node_parameter, key_name_list, value);
            break;
          }
        case rclcpp::ParameterType::PARAMETER_INTEGER:
          {
            int64_t value = parameter.as_int();
            updateConfigParam(node_parameter, key_name_list, value);
            break;
          }
        case rclcpp::ParameterType::PARAMETER_DOUBLE:
          {
            double value = parameter.as_double();
            updateConfigParam(node_parameter, key_name_list, value);
            break;
          }
        case rclcpp::ParameterType::PARAMETER_STRING:
          {
            std::string value = parameter.as_string();
            updateConfigParam(node_parameter, key_name_list, value);
            break;
          }
        case rclcpp::ParameterType::PARAMETER_BYTE_ARRAY:
          {
            auto byte_array = parameter.as_byte_array();
            if (byte_array.size() == 0) {
              RCLCPP_WARN(
              this->get_logger(),
              "parameter %s value is empty, it will not be stored", name.c_str());
              break;
            }
            YAML::Node seq;
            seq.SetStyle(YAML::EmitterStyle::Flow);
            for (auto byte : byte_array) {
              // TODO(fujitatomoya): rcl_yaml_param_parser does not support byte array,
              // use int array temporarily
              seq.push_back((int64_t)byte);
            }
            updateConfigParam(node_parameter, key_name_list, seq);
            break;
          }
        case rclcpp::ParameterType::PARAMETER_BOOL_ARRAY:
          {
            auto bool_array = parameter.as_bool_array();
            if (bool_array.size() == 0) {
              RCLCPP_WARN(
              this->get_logger(),
              "parameter %s value is empty, it will not be stored", name.c_str());
              break;
            }
            YAML::Node seq;
            seq.SetStyle(YAML::EmitterStyle::Flow);
            for (bool b : bool_array) {  // Vector is specialized for bool
              seq.push_back(b);
            }

            updateConfigParam(node_parameter, key_name_list, seq);
            break;
          }
        case rclcpp::ParameterType::PARAMETER_INTEGER_ARRAY:
          {
            auto array = parameter.as_integer_array();
            if (array.size() == 0) {
              RCLCPP_WARN(
              this->get_logger(),
              "parameter %s value is empty, it will not be stored", name.c_str());
              break;
            }
            YAML::Node seq;
            seq.SetStyle(YAML::EmitterStyle::Flow);
            for (auto i : array) {
              seq.push_back(i);
            }
            updateConfigParam(node_parameter, key_name_list, seq);
            break;
          }
        case rclcpp::ParameterType::PARAMETER_DOUBLE_ARRAY:
          {
            auto array = parameter.as_double_array();
            if (array.size() == 0) {
              RCLCPP_WARN(
              this->get_logger(),
              "parameter %s value is empty, it will not be stored", name.c_str());
              break;
            }
            YAML::Node seq;
            seq.SetStyle(YAML::EmitterStyle::Flow);
            for (auto d : array) {
              seq.push_back(d);
            }
            updateConfigParam(node_parameter, key_name_list, seq);
            break;
          }
        case rclcpp::ParameterType::PARAMETER_STRING_ARRAY:
          {
            auto array = parameter.as_string_array();
            if (array.size() == 0) {
              RCLCPP_WARN(
              this->get_logger(),
              "parameter %s value is empty, it will not be stored", name.c_str());
              break;
            }
            YAML::Node seq;
            seq.SetStyle(YAML::EmitterStyle::Flow);
            for (auto & str : array) {
              seq.push_back(str);
            }
            updateConfigParam(node_parameter, key_name_list, seq);
            break;
          }
        default:
          {
            RCLCPP_WARN(
            this->get_logger(),
            "parameter %s unsupported type %d",
            name.c_str(), parameter.get_type());
            break;
          }
      }
    }

    // rcl_yaml_param_parser not supported the following format, need to remove it
    //  /:
    //    parameter_server_bdk:
    //      ros__parameters:
    //        persistent:
    //          {}
    if (node_parameter[PERSISTENT_KEY].size() == 0) {
      node_parameter.remove(PERSISTENT_KEY);
    }

    if (parameter_use_stars_) {
      parameter_config["/**"][ROS_PARAMETER_KEY] = node_parameter;
    } else if (parameter_ns_exist_ && parameter_name_exist_) {
      parameter_config[get_namespace()][node_name_][ROS_PARAMETER_KEY] = node_parameter;
    } else if (parameter_name_exist_) {
      parameter_config[node_name_][ROS_PARAMETER_KEY] = node_parameter;
    } else {
      parameter_config[get_namespace()][node_name_][ROS_PARAMETER_KEY] = node_parameter;
    }

    // data -> YAML::Node -> YAML::Emitter(save string with "'")
    // use emitter to traverse all sub nodes, if value of Node is string, add ' between value.
    YAML::Emitter out;
    SaveNode(out, parameter_config);
    std::ofstream fout(persistent_yaml_file_);
    fout << out.c_str();
    fout.close();
  }
}

bool ParameterServer::CheckPersistentParam(const std::vector<rclcpp::Parameter> & parameters)
{
  RCLCPP_DEBUG(this->get_logger(), "%s", PARAMETER_SERVER_FUNCTION);
  bool flag = false;

  for (auto & parameter : parameters) {
    std::string parameter_name = parameter.get_name();
    if (parameter_name.find(PERSISTENT_DOT_KEY) != 0) {
      continue;
    }

    rclcpp::ParameterType parameter_type = parameter.get_type();
    if (rclcpp::ParameterType::PARAMETER_NOT_SET == parameter_type) {
      RCLCPP_DEBUG(this->get_logger(), "parameter %s not set", parameter_name.c_str());
      changed_parameter_lists_.erase(parameter_name);
      flag = true;
    } else {
      RCLCPP_DEBUG(this->get_logger(), "parameter %s changed", parameter_name.c_str());
      changed_parameter_lists_.insert(parameter_name);
      flag = true;
    }
  }

  return flag;
}
