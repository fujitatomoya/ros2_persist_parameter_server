// Copyright 2025 Sony Corporation
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

#ifndef __TEST_COMMON_H__
#define __TEST_COMMON_H__

#include <map>
#include <optional>
#include <stdexcept>
#include <type_traits>

#include "persist_parameter_client.hpp"

/**
 * NoServerError
 *
 * The client will wait 5 seconds for the server to be ready.
 * If timeout, then throw an exception to terminate the endless waiting.
 */
struct NoServerError : public std::runtime_error
{
public:
  NoServerError() : std::runtime_error("cannot connect to server") {}
};

/*
 * SetOperationError
 *
 * When executing `set_parameter`, if the set operation failed,
 * throw an exception to ignore the subsequent test.
 */
struct SetOperationError : public std::runtime_error
{
public:
  SetOperationError() : std::runtime_error("set operation failed") {}
};

class TestPersistParameter
{
public:
  TestPersistParameter(const std::string & node_name, const rclcpp::NodeOptions & options)
  : persist_param_client_(node_name, options)
  {
    if (!wait_param_server_ready()) {
      throw NoServerError();
    }
  }

  inline bool wait_param_server_ready() { return persist_param_client_.wait_param_server_ready(); }

  /*
    * Read the value of parameter.
    * @param param_name The name of parameter.
    * @param expected_value The value of the parameter that you expected, take std::string as example here.
    * If `expected_value` is `nullopt`, it means the parameter is expected not to exist.
    * @param testcase The test case description.
    */
  template <typename ValueType>
  void do_read_and_check(
    const std::string & param_name, const std::optional<ValueType> & expected_value,
    const std::string & testcase)
  {
    bool value_matches = false;
    std::vector<rclcpp::Parameter> parameter;

    if (persist_param_client_.read_parameter(param_name, parameter)) {
      for (auto & param : parameter) {
        if (!expected_value.has_value()) {
          if (param.get_type() == rclcpp::ParameterType::PARAMETER_NOT_SET) {
            value_matches = true;
            break;
          }
        } else {
          switch (param.get_type()) {
            case rclcpp::ParameterType::PARAMETER_STRING:
              if constexpr (std::is_same_v<ValueType, std::string>) {
                if (param.as_string() == expected_value.value()) {
                  value_matches = true;
                }
              }
              break;
            case rclcpp::ParameterType::PARAMETER_INTEGER:
              if constexpr (std::is_integral_v<ValueType>) {
                if (param.as_int() == expected_value.value()) {
                  value_matches = true;
                }
              }
              break;
            case rclcpp::ParameterType::PARAMETER_DOUBLE:
              if constexpr (std::is_floating_point_v<ValueType>) {
                if (
                  abs(param.as_double() - expected_value.value()) <
                  std::numeric_limits<double>::epsilon()) {
                  value_matches = true;
                }
              }
              break;
            // One might extend here for other types if needed (e.g., bool, double)
            default:
              break;
          }
        }
      }
    }

    /*
       * Even if the Get operation failed, record it in result_map, and it shouldn't effect the
       * subsequent tests.
       */
    this->set_result(testcase, value_matches);
  }

  /*
    * Change the value of parameter.
    * @param param_name The name of parameter.
    * @param changed_value The value that you want to set.
    * @param testcase The test case description.
    */
  template <typename ValueType>
  void do_change_and_check(
    const std::string & param_name, const ValueType & changed_value, const std::string & testcase)
  {
    bool ret = persist_param_client_.modify_parameter<ValueType>(param_name, changed_value);
    /*
       * If the Modify operation failed, record it in result_map, and no need to run the
       * subsequent read tests.
       */
    if (!ret) {
      this->set_result(testcase, false);
      throw SetOperationError();
    }

    do_read_and_check(param_name, std::make_optional(changed_value), testcase);
  }

  template <typename ValueType>
  void do_fail_to_change(
    const std::string & param_name, const ValueType & attempted_value, const std::string & testcase)
  {
    bool ret = persist_param_client_.modify_parameter<ValueType>(param_name, attempted_value);

    // this must fail. So set result to true if ret is false
    this->set_result(testcase, !ret);
  }

  /*
  * Used to check that reloading works. If save hasn't been called, parameters should be overwritten.
  * @param param_name The name of parameter.
  * @param changed_value The value that you want to set.
  * @param testcase The test case description.
  */
 template <typename ValueType>
  void do_reload_and_check(const std::string & param_name, const ValueType & changed_value, const std::optional<ValueType> & expected_value, const std::string & testcase) {      
    bool ret = false;

    ret = persist_param_client_.modify_parameter<std::string>(param_name, changed_value);
    /*
    * If the Modify operation failed, record it in result_map, and no need to run the 
    * subsequent read tests.
    */
    if(!ret) {
      this->set_result(testcase, false);
      throw SetOperationError();
    }

    /**
     * Attempt to reload the YAML file
     */
    auto reload_res = persist_param_client_.reload_yaml();
    if(!reload_res || !reload_res->success) {
      this->set_result(testcase, false);
      throw SetOperationError();
    }

    return do_read_and_check<ValueType>(param_name, expected_value, testcase);
  }

  /*
  * Change the value of parameter, save, read, then check.
  * @param param_name The name of parameter.
  * @param changed_value The value that you want to set.
  * @param testcase The test case description.
  */
 template <typename ValueType>
  void do_save_and_check(const std::string & param_name, const ValueType & changed_value, const std::string & testcase) {
    bool ret = false;

    ret = persist_param_client_.modify_parameter<std::string>(param_name, changed_value);
    /*
    * If the Modify operation failed, record it in result_map, and no need to run the 
    * subsequent read tests.
    */
    if(!ret) {
      this->set_result(testcase, false);
      throw SetOperationError();
    }

    /**
     * Manually trigger a save, if it returns false then there must have been an error in saving.
     */
    auto save_res = persist_param_client_.trigger_save();
    if(!save_res || !save_res->success) {
      this->set_result(testcase, false);
      throw SetOperationError();
    }

    /**
     * Attempt to reload the YAML file
     */
    auto reload_res = persist_param_client_.reload_yaml();
    if(!reload_res || !reload_res->success) {
      this->set_result(testcase, false);
      throw SetOperationError();
    }

    return do_read_and_check<ValueType>(param_name, changed_value, testcase);
  }

  // Get all test results.
  inline int print_result() const
  {
    int ret = EXIT_SUCCESS;
    RCLCPP_INFO(
      this->get_logger(),
      "****************************************************"
      "***********************");
    RCLCPP_INFO(
      this->get_logger(),
      "*********************************Test Result*********"
      "**********************");
    for (const auto & res : result_map_) {
      RCLCPP_INFO(
        this->get_logger(), "%-60s : %16s", res.first.c_str(), res.second ? "PASS" : "NOT PASS");

      // if any tests are not passed, return EXIT_FAILURE.
      if (res.second == false) {
        ret = EXIT_FAILURE;
      }
    }

    return ret;
  }

  static inline rclcpp::Logger get_logger() { return client_logger_; }

private:
  // Save the result of each test operation.
  inline void set_result(const std::string & key, bool value)
  {
    auto pair = result_map_.insert({key, value});
    if (!pair.second) {
      RCLCPP_INFO(this->get_logger(), "Failed when insert %s to result_map", key.c_str());
    }

    return;
  }

  PersistParametersClient persist_param_client_;
  std::map<std::string, bool> result_map_;
  static rclcpp::Logger client_logger_;
};

#endif
