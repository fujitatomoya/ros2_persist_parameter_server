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

#include <map>
#include <optional>
#include <stdexcept>
#include <type_traits>

#include "persist_parameter_client.hpp"
#include "test_common.h"

rclcpp::Logger TestPersistParameter::client_logger_ = rclcpp::get_logger("client");

// this test must be run somultaneously with the server node launched with the option allow-dynamic-typing set to true
int main(int argc, char ** argv)
{
  // force flush of the stdout buffer.
  // this ensures a correct sync of all prints
  // even when executed simultaneously within the launch file.
  setvbuf(stdout, NULL, _IONBF, BUFSIZ);

  rclcpp::init(argc, argv);
  std::shared_ptr<TestPersistParameter> test_client;

  RCLCPP_INFO(test_client->get_logger(), "****************************************************"
                                    "***********************");
  int ret_code = 0;
  try {
    test_client = std::make_shared<TestPersistParameter>("client", rclcpp::NodeOptions());
    /*
    * Dynamic Typing Tests
    *
    * These tests will try to change the type of a parameter to see whether the
    * server accepts it based on allow_dynamic_typing=true/false.
    */
    {
      test_client->do_change_and_check<std::string>(
        "persistent.some_int", "mutated", "a. dynamically change the type of an existing parameter");
      test_client->do_change_and_check<int>(
        "persistent.some_int", 10, "b. revert the type of the parameter to int");
      test_client->do_change_and_check<double>(
        "persistent.new_double", 3.14, "c. create new parameter with type double");
      test_client->do_change_and_check<std::string>(
        "persistent.new_double", "3.14", "d. change the type of the new parameter to string");
    }

  } catch (const rclcpp::exceptions::RCLError & e) {
    ret_code = -1;
    RCLCPP_ERROR(test_client->get_logger(), "unexpectedly failed: %s", e.what());
  } catch (const NoServerError & e) {
    ret_code = -2;
    RCLCPP_ERROR(test_client->get_logger(), "unexpectedly failed: %s", e.what());
  } catch (const SetOperationError & e) {
    ret_code = -3;
    RCLCPP_ERROR(test_client->get_logger(), "unexpectedly failed: %s", e.what());
  }

  // if any tests are not passed, return EXIT_FAILURE.
  ret_code = test_client->print_result();
  rclcpp::shutdown();

  return ret_code;
}
