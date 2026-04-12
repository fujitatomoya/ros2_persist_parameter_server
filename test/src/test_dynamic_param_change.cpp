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

#include <map>
#include <optional>
#include <stdexcept>
#include <type_traits>

#include "persist_parameter_client.hpp"
#include "test_common.h"

rclcpp::Logger TestPersistParameter::client_logger_ = rclcpp::get_logger("client");

// This test must be run simultaneously with the server node launched with
int main(int argc, char ** argv)
{
  // force flush of the stdout buffer.
  // this ensures a correct sync of all prints
  // even when executed simultaneously within the launch file.
  setvbuf(stdout, NULL, _IONBF, BUFSIZ);

  rclcpp::init(argc, argv);
  std::shared_ptr<TestPersistParameter> test_client;

  int ret_code = 0;
  try {
    test_client = std::make_shared<TestPersistParameter>("client", rclcpp::NodeOptions());

    {
      RCLCPP_INFO(test_client->get_logger(), "Test save_on_update param dynamic turn on");

      // Start with save on update at false
      test_client->do_read_server_param_and_check<bool>("must_save_on_update",
        false, "a. Check initial must_save_on_update value");
      // Modify the parameter and check that it is changed
      test_client->do_server_param_change_and_check<bool>("must_save_on_update",
        true, true, "b. Dynamically enable save on update");
      // Check that a modified value is saved on update
      test_client->do_change_and_check<std::string>(
        "a_string", std::string{"Hi"}, "c. Check that save on update works");
    }

    {
      RCLCPP_INFO(test_client->get_logger(), "Test save_on_update param dynamic turn off");
      // Start with save on update at true
      test_client->do_read_server_param_and_check<bool>("must_save_on_update",
        true, "d. Check initial must_save_on_update value");
      // Modify the parameter and check that it is changed
      test_client->do_server_param_change_and_check<bool>("must_save_on_update",
        false, false, "e. Dynamically disable save on update");
      // Check that a modified value is not saved on update
      test_client->do_reload_and_check<std::string>(
        "a_string", std::string{"there"}, std::string{"Hi"},
        "f. Check that change is saved on update");
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
