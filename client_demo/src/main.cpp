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

#include "persist_parameter_client.hpp"
#include <map>
#include <exception>

/* NoServerException
 *
 * The client will wait 5 seconds for the server to be ready.
 * If not, then throw an exception to terminate the endless waiting.
 */
struct NoServerException : public std::exception
{
  const char * what () const throw () {
    return "Not find server";
  }
};

/* SetErrorException
 *
 * When executing `set_parameter`, if the set operation failed, 
 * throw an exception to ignore the subsequent test.
 */
struct SetErrorException : public std::exception
{
  const char * what () const throw () {
    return "set operation failed";
  }
};

class TestPersistParameter
{
  public:
  	TestPersistParameter(
      const std::string & node_name, 
      const rclcpp::NodeOptions & options)
      : persist_param_client_(node_name, options)
    {
      if(!wait_param_server_ready()) {
        throw NoServerException();
      }
    }

    inline bool wait_param_server_ready()
    {
      return persist_param_client_.wait_param_server_ready();
    }

    /*
    * Read the value of parameter.
    * @param param_name The name of parameter.
    * @param expect_str The value of the parameter that you expected, take std::string as example here.
    * If expect_str is equal to `nullptr` pointer, that means the parameter expected to be not exist.
    * @param testcase The test case description.
    */
    void do_read_and_check(const std::string & param_name, const char * expect_str, const std::string & testcase)
    {
      bool value = false;
      std::vector<rclcpp::Parameter> parameter;

      if(persist_param_client_.read_parameter(param_name, parameter)) {
        for(auto & param : parameter) {
          if(expect_str == nullptr) {
            if(param.get_type() == rclcpp::ParameterType::PARAMETER_NOT_SET) {
              value = true;
              break;
            }
          }else if(param.get_type() == rclcpp::ParameterType::PARAMETER_STRING && param.as_string() == expect_str) {
            value = true;
            break;
          }
        }
      }

      /*
       * Even if the Get operation failed, record it in result_map, and it shouldn't effect the
       * subsequent tests.
       */
      this->set_result(testcase, value);
    }

    /*
    * Change the value of parameter.
    * @param param_name The name of parameter.
    * @param changed_value The value that you want to set.
    * @param testcase The test case description.
    */
    void do_change_and_check(const std::string & param_name, const std::string & changed_value, const std::string & testcase)
    {
      bool ret = false;

      ret = persist_param_client_.modify_parameter<std::string>(param_name, changed_value);
      /*
       * If the Modify operation failed, record it in result_map, and no need to run the 
       * subsequent read tests.
       */
      if(!ret) {
        this->set_result(testcase, false);
        throw SetErrorException();
      }

      return do_read_and_check(param_name, changed_value.c_str(), testcase);
    }

    // Get all test results.
    inline void get_result() const
    {
      RCLCPP_INFO(get_logger("client"), "****************************************************"
                                        "***********************");
      RCLCPP_INFO(get_logger("client"), "*********************************Test Result*********"
                                        "**********************");
      for(const auto & res : result_map) {
        RCLCPP_INFO(get_logger("client"), "%s : \t\t%s", res.first.c_str(), res.second?"PASS":"NOT PASS");
      }

      return;
    }

private:
    // Save the result of each test operation.
    inline void set_result(std::string key, bool value)
    {
      auto pair = result_map.insert(std::make_pair<std::string, bool>(std::move(key), std::move(value)));
      if(!pair.second) {
        RCLCPP_INFO(get_logger("client"), "Failed when insert %s to result_map", key.c_str());
      }

      return;
    }

    PersistParametersClient persist_param_client_;
    std::map<std::string, bool> result_map;
};
	
int main(int argc, char ** argv)
{
  // force flush of the stdout buffer.
  // this ensures a correct sync of all prints
  // even when executed simultaneously within the launch file.
  setvbuf(stdout, NULL, _IONBF, BUFSIZ);

  rclcpp::init(argc, argv);

  // In case of an exception is thrown when performing an operation after `ctrl-c` occured.
  try {
    auto test_client = std::make_shared<TestPersistParameter>("client", rclcpp::NodeOptions());
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
      RCLCPP_INFO(get_logger("client"), "First read the initial value of parameter : ");
      test_client->do_read_and_check("a_string", "Hello world", "a. Read Normal Parameter");
      test_client->do_read_and_check("persistent.a_string", "Hello world", "b. Read Persistent Parameter");
    }

    /*
    * Test: Modifying the parameter `a_string`'s value to `Hello`, and add a new parameter `new_string` to YAML file. 
    */
    {
      RCLCPP_INFO(get_logger("client"), "Change the value of parameter to `Hello` : ");
      test_client->do_change_and_check("a_string", "Hello", "c. Modify Existed Normal parameter");
      test_client->do_change_and_check("persistent.a_string", "Hello", "d. Modify Existed Persistent parameter");
      RCLCPP_INFO(get_logger("client"), "Add a new parameter to parameter file : ");
      test_client->do_change_and_check("new_string", "Hello NewString", "e. Add New Normal parameter");
      test_client->do_change_and_check("persistent.new_string", "Hello NewString", "f. Add New Persistent parameter");
    }

    // Waiting for the server to restart.
    std::this_thread::sleep_for(std::chrono::seconds(5));

    /*
    * Test : Reading parameter value again to confirm whether to store the modified persistent/normal parameter to the file.
    */
    {
      if(!test_client->wait_param_server_ready()) {
        throw NoServerException();
      }
      RCLCPP_INFO(get_logger("client"), "Last read the value of parameter after server restarts," 
        "to check whether changes stores to the file : ");
      test_client->do_read_and_check("a_string", "Hello world", "g. Test Normal Parameter Not Stores To File");
      test_client->do_read_and_check("persistent.a_string", "Hello", "h. Test Persistent Parameter Stores To File");
	  test_client->do_read_and_check("new_string", nullptr, "i. Test New Added Normal Parameter Not Stores To File");
      test_client->do_read_and_check("persistent.new_string", "Hello NewString", "j. Test New Added Persistent Parameter Stores To File");
    }

    test_client->get_result();
  }catch(const rclcpp::exceptions::RCLError & e) {
    RCLCPP_ERROR(get_logger("rcl"), "unexpectedly failed: %s", e.what());
  }catch(const NoServerException & e) {
    RCLCPP_ERROR(get_logger("client"), "unexpectedly failed: %s", e.what());
  }catch(const SetErrorException & e) {
    RCLCPP_ERROR(get_logger("client"), "unexpectedly failed: %s", e.what());
  }

  rclcpp::shutdown();

  return 0;
}
