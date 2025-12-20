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

int main(int argc, char ** argv)
{
  // force flush of the stdout buffer.
  // this ensures a correct sync of all prints
  // even when executed simultaneously within the launch file.
  setvbuf(stdout, NULL, _IONBF, BUFSIZ);

  rclcpp::init(argc, argv);
  std::shared_ptr<TestPersistParameter> test_client;

  int ret_code = 0;
  // In case of an exception is thrown when performing an operation after `ctrl-c` occurred.
  try {
    test_client = std::make_shared<TestPersistParameter>("client", rclcpp::NodeOptions());
    /*
    * First read parameter(include normal parameter and persistent parameter), to confirm the initial value of parameters.
    * Parameter server is launched with file `/tmp/parameter_server.yaml`, in this file, parameter `a_string` is defined
    * and the initial value is `Hello world`.
    *
    * Test: The default parameter `a_string` specified in YAML file which is loaded while parameter server is launched
    * should be read correctly.
    */
    {
      // If return fail, no need to do the following.
      RCLCPP_INFO(test_client->get_logger(), "First read the initial value of parameter : ");
      test_client->do_read_and_check<std::string>(
        "a_string", "Hello world", "a. Read Normal Parameter");
      test_client->do_read_and_check<std::string>(
        "persistent.a_string", "Hello world", "b. Read Persistent Parameter");
    }

    /*
    * Test: Modifying the parameter `a_string`'s value to `Hello`, and add a new parameter `new_string` to YAML file.
    */
    {
      RCLCPP_INFO(test_client->get_logger(), "Change the value of parameter to `Hello` : ");
      test_client->do_change_and_check<std::string>(
        "a_string", std::string{"Hello"}, "c. Modify Existed Normal parameter");
      test_client->do_change_and_check<std::string>(
        "persistent.a_string", std::string{"Hello"}, "d. Modify Existed Persistent parameter");
      RCLCPP_INFO(test_client->get_logger(), "Add a new parameter to parameter file : ");
      test_client->do_change_and_check<std::string>(
        "new_string", std::string{"Hello NewString"}, "e. Add New Normal parameter");
      test_client->do_change_and_check<std::string>(
        "persistent.new_string", std::string{"Hello NewString"}, "f. Add New Persistent parameter");
    }

    // Waiting for the server to restart.
    std::this_thread::sleep_for(std::chrono::seconds(5));

    /*
    * Test : Reading parameter value again to confirm whether to store the modified persistent/normal parameter to the file.
    */
    {
      if (!test_client->wait_param_server_ready()) {
        throw NoServerError();
      }
      RCLCPP_INFO(
        test_client->get_logger(),
        "Last read the value of parameter after server restarts,"
        "to check whether changes stores to the file : ");
      test_client->do_read_and_check<std::string>(
        "a_string", "Hello world", "g. Test Normal Parameter Not Stores To File");
      test_client->do_read_and_check<std::string>(
        "persistent.a_string", "Hello", "h. Test Persistent Parameter Stores To File");
      test_client->do_read_and_check<std::string>(
        "new_string", std::nullopt, "i. Test New Added Normal Parameter Not Stores To File");
      test_client->do_read_and_check<std::string>(
        "persistent.new_string", "Hello NewString",
        "j. Test New Added Persistent Parameter Stores To File");
    }

    /*
    * Test : Impossible to change the type of a parameter.
    */
    {
      RCLCPP_INFO(
        test_client->get_logger(), "Try to change the type of a parameter, must not be possible:");
      test_client->do_fail_to_change<int>(
        "persistent.a_string", 10, "k. Test could not change the type of persistent parameter");
      test_client->do_fail_to_change<std::string>(
        "some_int", "Not possible", "l. Test could not change the type of parameter");
    }

    /*
    * Test: Modifying parameters the same as above but saving the file and then checking. 
    */
    {
      RCLCPP_INFO(test_client->get_logger(), "Change the value of parameter to `Hello` : ");
      test_client->do_save_and_check<std::string>("persistent.test_saved_first", "Hello", "m. Set and saved new parameter successfully");
      RCLCPP_INFO(test_client->get_logger(), "Add a new parameter to parameter file : ");
      test_client->do_save_and_check<std::string>("persistent.test_saved_second", "SecondString", "n. Set and saved new parameter successfully");
      RCLCPP_INFO(test_client->get_logger(), "Update and save a parameter and make sure it saved successfully : ");
      test_client->do_save_and_check<std::string>("persistent.test_saved_first", "World", "o. Set and saved existing parameter successfully");
      RCLCPP_INFO(test_client->get_logger(), "Update a parameter and reload without saving : ");
      test_client->do_reload_and_check<std::string>("persistent.test_saved_second", "Discarded", "SecondString", "p. Set and saved new parameter successfully");
    }

    /*
    * Test: Double precision handling (Issue #13)
    * Test that double values maintain their precision through save/reload cycles.
    */
    {
      RCLCPP_INFO(test_client->get_logger(), "Testing double precision handling:");
      // Test high precision double value
      double high_precision_value = 3.141592653589793;
      test_client->do_save_and_check<double>(
        "persistent.high_precision_double", high_precision_value,
        "q. Set and saved high precision double successfully");

      // Test another high precision value with many decimal places
      double scientific_value = 1.23456789012345e-10;
      test_client->do_save_and_check<double>(
        "persistent.scientific_notation_double", scientific_value,
        "r. Set and saved double in scientific notation successfully");

      // Test double that could lose precision (e.g., 10.0 should not become 10)
      double trailing_zero_value = 10.0;
      test_client->do_save_and_check<double>(
        "persistent.trailing_zero_double", trailing_zero_value,
        "s. Set and saved double with trailing zero successfully");
    }

    /*
    * Test: Double array precision handling (Issue #13)
    * Test that double arrays maintain their precision through save/reload cycles.
    */
    {
      RCLCPP_INFO(test_client->get_logger(), "Testing double array precision handling:");
      // Test double array with high precision values
      std::vector<double> high_precision_array = {
        3.141592653589793, 2.718281828459045, 1.41421356237309504880168872420969807856967187537694
      };
      test_client->do_save_and_check<std::vector<double>>(
        "persistent.high_precision_double_array", high_precision_array,
        "t. Set and saved high precision double array successfully");

      // Test double array with mixed notation values
      std::vector<double> mixed_array = {10.0, 1.5e-8, 999.999999, 0.000000001};
      test_client->do_save_and_check<std::vector<double>>(
        "persistent.mixed_notation_double_array", mixed_array,
        "u. Set and saved mixed notation double array successfully");
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
