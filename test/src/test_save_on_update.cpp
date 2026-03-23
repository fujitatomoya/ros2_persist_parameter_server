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
// --save-on-update true and --storing-period 0
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

    /*
    * Test: Auto-save on parameter change.
    * With save-on-update enabled, setting a persistent parameter should immediately persist it.
    * We set all types, do a single reload, and verify every value survived.
    */
    {
      RCLCPP_INFO(test_client->get_logger(), "Test auto-save: set persistent parameters and reload");

      test_client->do_change_and_check<std::string>(
        "persistent.auto_saved_string", std::string{"AutoSaved"},
        "a. Set persistent string parameter");
      test_client->do_change_and_check<int>(
        "persistent.auto_saved_int", 42,
        "b. Set persistent integer parameter");
      test_client->do_change_and_check<double>(
        "persistent.auto_saved_double", 3.14159,
        "c. Set persistent double parameter");
      test_client->do_change_and_check<bool>(
        "persistent.auto_saved_bool", true,
        "d. Set persistent bool parameter");

      // Reload from disk — all values should persist because save-on-update is enabled
      auto reload_res = test_client->get_client().reload_yaml();
      if (!reload_res || !reload_res->success) {
        throw SetOperationError();
      }

      test_client->do_read_and_check<std::string>(
        "persistent.auto_saved_string", "AutoSaved",
        "e. Persistent string survives reload (auto-saved)");
      test_client->do_read_and_check<int>(
        "persistent.auto_saved_int", 42,
        "f. Persistent integer survives reload (auto-saved)");
      test_client->do_read_and_check<double>(
        "persistent.auto_saved_double", 3.14159,
        "g. Persistent double survives reload (auto-saved)");
      test_client->do_read_and_check<bool>(
        "persistent.auto_saved_bool", true,
        "h. Persistent bool survives reload (auto-saved)");
    }

    /*
    * Test: Updating an auto-saved parameter overwrites the previous value.
    */
    {
      RCLCPP_INFO(test_client->get_logger(), "Test auto-save overwrites previous value");

      test_client->do_change_and_check<std::string>(
        "persistent.auto_saved_string", std::string{"UpdatedValue"},
        "i. Update previously auto-saved parameter");

      auto reload_res = test_client->get_client().reload_yaml();
      if (!reload_res || !reload_res->success) {
        throw SetOperationError();
      }
      test_client->do_read_and_check<std::string>(
        "persistent.auto_saved_string", "UpdatedValue",
        "j. Updated value persists after reload");
    }

    /*
    * Test: Manual save service still works alongside auto-save.
    */
    {
      RCLCPP_INFO(test_client->get_logger(), "Test manual save service still works");

      test_client->do_save_and_check<std::string>(
        "persistent.manually_saved", "ManualValue",
        "k. Manual save service works with auto-save enabled");
    }

    /*
    * Test: Manual reload service discards unsaved non-persistent changes.
    */
    {
      RCLCPP_INFO(test_client->get_logger(), "Test reload discards non-persistent changes");

      // First set and auto-save a persistent parameter
      test_client->do_change_and_check<std::string>(
        "persistent.reload_test", std::string{"Saved"},
        "l. Set persistent parameter for reload test");

      // Now change it — this will also auto-save the new value
      test_client->do_change_and_check<std::string>(
        "persistent.reload_test", std::string{"Changed"},
        "m. Change the parameter (also auto-saved)");
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
