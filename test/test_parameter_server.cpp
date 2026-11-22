// Copyright 2026 Tomoya Fujita
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

#include <gtest/gtest.h>

#include <atomic>
#include <chrono>
#include <filesystem>
#include <fstream>
#include <limits>
#include <memory>
#include <random>
#include <sstream>
#include <stdexcept>
#include <string>
#include <thread>
#include <type_traits>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "yaml-cpp/yaml.h"

#include "parameter_server.hpp"
#include "persist_parameter_client.hpp"

using namespace std::chrono_literals;

namespace
{

/// Same content as server/param/parameter_server.yaml, used as the initial persistent file.
constexpr char kDefaultYaml[] =
  "/**:\n"
  "    ros__parameters:\n"
  "        some_int: 1\n"
  "        a_string: \"Hello world\"\n"
  "        pi: 3.14\n"
  "        some_lists:\n"
  "            some_integers: [1, 2, 3, 4]\n"
  "        persistent:\n"
  "            some_int: 1\n"
  "            a_string: 'Hello world'\n"
  "            pi: 3.14\n"
  "            some_lists:\n"
  "                some_integers: [1, 2, 3, 4]\n";

void write_file(const std::string & path, const std::string & content)
{
  std::ofstream out(path);
  out << content;
}

std::string unique_test_directory()
{
  std::random_device rd;
  std::ostringstream ss;
  ss << "persist_parameter_server_test_" << std::hex << rd();
  return (std::filesystem::temp_directory_path() / ss.str()).string();
}

}  // namespace

/// Fixture running the parameter server in-process with a dedicated executor thread.
/**
 * The server node is spun on a background thread, while the test body drives a
 * PersistParametersClient node to read/write parameters exactly as an external
 * node would do. Stopping the server destroys the node, which flushes the persistent
 * parameters into the yaml file, so restarting the server behaves like a process restart.
 */
class ParameterServerTest : public ::testing::Test
{
protected:
  static void SetUpTestSuite()
  {
    rclcpp::init(0, nullptr);
  }

  static void TearDownTestSuite()
  {
    rclcpp::shutdown();
  }

  void SetUp() override
  {
    test_dir_ = unique_test_directory();
    std::filesystem::create_directories(test_dir_);
    yaml_file_ = (std::filesystem::path(test_dir_) / "parameter_server.yaml").string();
    write_file(yaml_file_, kDefaultYaml);
  }

  void TearDown() override
  {
    stop_server();
    client_.reset();
    std::filesystem::remove_all(test_dir_);
  }

  /// Start the parameter server node with the given options and wait until it is ready.
  void start_server(
    bool allow_dynamic_typing = false, int storing_period = 0, bool save_on_update = false,
    bool allow_override = true)
  {
    ASSERT_EQ(server_, nullptr) << "server is already running";

    rclcpp::NodeOptions options = rclcpp::NodeOptions()
      .allow_undeclared_parameters(true)
      .automatically_declare_parameters_from_overrides(allow_override)
      .append_parameter_override("allow_dynamic_typing", allow_dynamic_typing)
      .append_parameter_override("storing_period", storing_period)
      .append_parameter_override("must_save_on_update", save_on_update);
    for (const auto & override : extra_overrides_) {
      options.append_parameter_override(override.get_name(), override.get_parameter_value());
    }
    if (!server_namespace_.empty()) {
      options.arguments({"--ros-args", "-r", "__ns:=" + server_namespace_});
    }

    server_ = std::make_shared<ParameterServer>("parameter_server", options, yaml_file_);
    executor_ = std::make_unique<rclcpp::executors::SingleThreadedExecutor>();
    executor_->add_node(server_);
    spinning_ = true;
    spin_thread_ = std::thread(
      [this]() {
        while (spinning_) {
          executor_->spin_once(100ms);
        }
      });

    if (!client_) {
      client_ = std::make_shared<PersistParametersClient>(
        "client", server_namespace_ + "/parameter_server");
    }
    ASSERT_TRUE(client_->wait_param_server_ready());
  }

  /// Stop the parameter server node, which stores the persistent parameters into the file.
  void stop_server()
  {
    if (spin_thread_.joinable()) {
      spinning_ = false;
      spin_thread_.join();
    }
    if (executor_ && server_) {
      executor_->remove_node(server_);
    }
    executor_.reset();
    server_.reset();
  }

  void restart_server(
    bool allow_dynamic_typing = false, int storing_period = 0, bool save_on_update = false,
    bool allow_override = true)
  {
    stop_server();
    start_server(allow_dynamic_typing, storing_period, save_on_update, allow_override);
  }

  /// Read `name` from the server and check that it holds `expected`.
  template<typename ValueType>
  void expect_value(const std::string & name, const ValueType & expected)
  {
    rclcpp::Parameter param = client_->read_parameter(name);
    ASSERT_NE(param.get_type(), rclcpp::ParameterType::PARAMETER_NOT_SET) << name;
    ValueType actual;
    ASSERT_NO_THROW(actual = param.get_value<ValueType>()) << name;
    if constexpr (std::is_floating_point_v<ValueType>) {
      EXPECT_DOUBLE_EQ(actual, expected) << name;
    } else {
      EXPECT_EQ(actual, expected) << name;
    }
  }

  void expect_not_set(const std::string & name)
  {
    rclcpp::Parameter param = client_->read_parameter(name);
    EXPECT_EQ(param.get_type(), rclcpp::ParameterType::PARAMETER_NOT_SET) << name;
  }

  /// Set `name` to `value` on the server and read it back.
  template<typename ValueType>
  void change_and_check(const std::string & name, const ValueType & value)
  {
    ASSERT_TRUE(client_->modify_parameter(name, value)) << name;
    expect_value(name, value);
  }

  /// Set `name` to `value`, save, reload and read it back from the server.
  template<typename ValueType>
  void save_and_check(const std::string & name, const ValueType & value)
  {
    ASSERT_TRUE(client_->modify_parameter(name, value)) << name;
    ASSERT_TRUE(client_->trigger_save());
    ASSERT_TRUE(client_->reload_params());
    expect_value(name, value);
  }

  /// Set `name` to `value` without saving, reload, and expect `expected` from the file.
  template<typename ValueType>
  void reload_and_check(
    const std::string & name, const ValueType & value, const ValueType & expected)
  {
    ASSERT_TRUE(client_->modify_parameter(name, value)) << name;
    ASSERT_TRUE(client_->reload_params());
    expect_value(name, expected);
  }

  YAML::Node persistent_node_in_file(const std::string & node_key = "/**")
  {
    YAML::Node config = YAML::LoadFile(yaml_file_);
    return config[node_key]["ros__parameters"]["persistent"];
  }

  /// Wait until the persistent parameter `key` holds `expected` in the yaml file.
  /**
   * With save-on-update enabled, the server stores the file asynchronously from the
   * parameter event callback on ROS 2 Humble (post-set parameter callbacks are only
   * available since Iron), so the file must be awaited before reloading or inspecting it.
   */
  template<typename ValueType>
  void wait_for_stored_value(const std::string & key, const ValueType & expected)
  {
    const auto deadline = std::chrono::steady_clock::now() + 5s;
    while (std::chrono::steady_clock::now() < deadline) {
      try {
        YAML::Node node = persistent_node_in_file()[key];
        if (node && node.as<ValueType>() == expected) {
          return;
        }
      } catch (const YAML::Exception &) {
        // the file does not hold the value yet
      }
      std::this_thread::sleep_for(20ms);
    }
    FAIL() << "persistent parameter " << key << " was not stored in " << yaml_file_;
  }

  std::string test_dir_;
  std::string yaml_file_;
  std::string server_namespace_;
  std::vector<rclcpp::Parameter> extra_overrides_;
  std::shared_ptr<ParameterServer> server_;
  std::unique_ptr<rclcpp::executors::SingleThreadedExecutor> executor_;
  std::thread spin_thread_;
  std::atomic_bool spinning_{false};
  std::shared_ptr<PersistParametersClient> client_;
};

TEST_F(ParameterServerTest, load_parameters_from_yaml_file)
{
  ASSERT_NO_FATAL_FAILURE(start_server());

  expect_value<int64_t>("some_int", 1);
  expect_value<std::string>("a_string", "Hello world");
  expect_value<double>("pi", 3.14);
  expect_value<std::vector<int64_t>>("some_lists.some_integers", {1, 2, 3, 4});

  expect_value<int64_t>("persistent.some_int", 1);
  expect_value<std::string>("persistent.a_string", "Hello world");
  expect_value<double>("persistent.pi", 3.14);
  expect_value<std::vector<int64_t>>("persistent.some_lists.some_integers", {1, 2, 3, 4});

  expect_not_set("does_not_exist");
}

TEST_F(ParameterServerTest, persistent_parameters_survive_restart)
{
  ASSERT_NO_FATAL_FAILURE(start_server());

  // modify existing parameters and add new ones, both normal and persistent.
  change_and_check<std::string>("a_string", "Hello");
  change_and_check<std::string>("persistent.a_string", "Hello");
  change_and_check<std::string>("new_string", "Hello NewString");
  change_and_check<std::string>("persistent.new_string", "Hello NewString");

  // restart the server: persistent parameters are stored on shutdown and loaded on startup.
  ASSERT_NO_FATAL_FAILURE(restart_server());

  expect_value<std::string>("a_string", "Hello world");
  expect_value<std::string>("persistent.a_string", "Hello");
  expect_not_set("new_string");
  expect_value<std::string>("persistent.new_string", "Hello NewString");

  // the file itself only carries the persistent changes.
  YAML::Node ros_parameters = YAML::LoadFile(yaml_file_)["/**"]["ros__parameters"];
  EXPECT_EQ(ros_parameters["a_string"].as<std::string>(), "Hello world");
  EXPECT_FALSE(ros_parameters["new_string"]);
  EXPECT_EQ(ros_parameters["persistent"]["a_string"].as<std::string>(), "Hello");
  EXPECT_EQ(ros_parameters["persistent"]["new_string"].as<std::string>(), "Hello NewString");
}

TEST_F(ParameterServerTest, parameter_type_change_is_rejected_by_default)
{
  ASSERT_NO_FATAL_FAILURE(start_server());

  EXPECT_FALSE(client_->modify_parameter("persistent.a_string", 10));
  EXPECT_FALSE(client_->modify_parameter("some_int", "Not possible"));

  expect_value<std::string>("persistent.a_string", "Hello world");
  expect_value<int64_t>("some_int", 1);
}

TEST_F(ParameterServerTest, save_and_reload_services)
{
  ASSERT_NO_FATAL_FAILURE(start_server());

  save_and_check<std::string>("persistent.test_saved_first", "Hello");
  save_and_check<std::string>("persistent.test_saved_second", "SecondString");
  save_and_check<std::string>("persistent.test_saved_first", "World");

  // a change which is not saved is discarded by reloading the file.
  reload_and_check<std::string>("persistent.test_saved_second", "Discarded", "SecondString");

  YAML::Node persistent = persistent_node_in_file();
  EXPECT_EQ(persistent["test_saved_first"].as<std::string>(), "World");
  EXPECT_EQ(persistent["test_saved_second"].as<std::string>(), "SecondString");
}

TEST_F(ParameterServerTest, double_precision_is_preserved)
{
  ASSERT_NO_FATAL_FAILURE(start_server());

  const double high_precision_value = 3.141592653589793;
  const double scientific_value = 1.23456789012345e-10;
  const double trailing_zero_value = 10.0;
  const std::vector<double> high_precision_array = {
    3.141592653589793, 2.718281828459045, 1.41421356237309504880168872420969807856967187537694};
  const std::vector<double> mixed_array = {10.0, 1.5e-8, 999.999999, 0.000000001};

  save_and_check("persistent.high_precision_double", high_precision_value);
  save_and_check("persistent.scientific_notation_double", scientific_value);
  save_and_check("persistent.trailing_zero_double", trailing_zero_value);
  save_and_check("persistent.high_precision_double_array", high_precision_array);
  save_and_check("persistent.mixed_notation_double_array", mixed_array);

  // the values must be kept as double in the file as well, e.g. 10.0 must not become 10.
  ASSERT_NO_FATAL_FAILURE(restart_server());

  expect_value("persistent.high_precision_double", high_precision_value);
  expect_value("persistent.scientific_notation_double", scientific_value);
  expect_value("persistent.trailing_zero_double", trailing_zero_value);
  expect_value("persistent.high_precision_double_array", high_precision_array);
  expect_value("persistent.mixed_notation_double_array", mixed_array);

  YAML::Node persistent = persistent_node_in_file();
  EXPECT_EQ(persistent["trailing_zero_double"].as<std::string>(), "10.0");
  EXPECT_DOUBLE_EQ(persistent["high_precision_double"].as<double>(), high_precision_value);
}

TEST_F(ParameterServerTest, dynamic_typing_allows_type_change)
{
  ASSERT_NO_FATAL_FAILURE(start_server(true));

  change_and_check<std::string>("persistent.some_int", "mutated");
  change_and_check<int64_t>("persistent.some_int", 10);
  change_and_check<double>("persistent.new_double", 3.14);
  change_and_check<std::string>("persistent.new_double", "3.14");

  ASSERT_NO_FATAL_FAILURE(restart_server(true));

  expect_value<int64_t>("persistent.some_int", 10);
  expect_value<std::string>("persistent.new_double", "3.14");
}

TEST_F(ParameterServerTest, save_on_update_stores_immediately)
{
  ASSERT_NO_FATAL_FAILURE(start_server(false, 0, true));

  change_and_check<std::string>("persistent.auto_saved_string", "AutoSaved");
  change_and_check<int64_t>("persistent.auto_saved_int", 42);
  change_and_check<double>("persistent.auto_saved_double", 3.14159);
  change_and_check<bool>("persistent.auto_saved_bool", true);
  ASSERT_NO_FATAL_FAILURE(wait_for_stored_value<bool>("auto_saved_bool", true));

  // reload from the file without saving: all values must have been auto-saved.
  ASSERT_TRUE(client_->reload_params());
  expect_value<std::string>("persistent.auto_saved_string", "AutoSaved");
  expect_value<int64_t>("persistent.auto_saved_int", 42);
  expect_value<double>("persistent.auto_saved_double", 3.14159);
  expect_value<bool>("persistent.auto_saved_bool", true);

  // updating an auto-saved parameter overwrites the previous value in the file.
  ASSERT_TRUE(
    client_->modify_parameter<std::string>(
      "persistent.auto_saved_string",
      "UpdatedValue"));
  ASSERT_NO_FATAL_FAILURE(
    wait_for_stored_value<std::string>("auto_saved_string", "UpdatedValue"));
  ASSERT_TRUE(client_->reload_params());
  expect_value<std::string>("persistent.auto_saved_string", "UpdatedValue");

  // manual save service keeps working alongside save-on-update.
  save_and_check<std::string>("persistent.manually_saved", "ManualValue");

  // normal parameters are never stored, even with save-on-update.
  ASSERT_TRUE(client_->modify_parameter("not_persistent", "Volatile"));
  ASSERT_TRUE(client_->reload_params());
  YAML::Node ros_parameters = YAML::LoadFile(yaml_file_)["/**"]["ros__parameters"];
  EXPECT_FALSE(ros_parameters["not_persistent"]);
  EXPECT_EQ(ros_parameters["persistent"]["auto_saved_int"].as<int64_t>(), 42);
}

TEST_F(ParameterServerTest, save_on_update_can_be_changed_dynamically)
{
  ASSERT_NO_FATAL_FAILURE(start_server(false, 0, false));

  // dynamically turn save-on-update on.
  expect_value<bool>("must_save_on_update", false);
  change_and_check<bool>("must_save_on_update", true);
  ASSERT_TRUE(client_->modify_parameter<std::string>("persistent.a_string", "Hi"));
  ASSERT_NO_FATAL_FAILURE(wait_for_stored_value<std::string>("a_string", "Hi"));
  ASSERT_TRUE(client_->reload_params());
  expect_value<std::string>("persistent.a_string", "Hi");

  // dynamically turn save-on-update off.
  change_and_check<bool>("must_save_on_update", false);
  reload_and_check<std::string>("persistent.a_string", "there", "Hi");
}

TEST_F(ParameterServerTest, storing_period_can_be_changed_dynamically)
{
  ASSERT_NO_FATAL_FAILURE(start_server(false, 0, false));

  // negative storing period is rejected.
  EXPECT_FALSE(client_->modify_parameter<int64_t>("storing_period", -1));
  expect_value<int64_t>("storing_period", 0);

  // enable the periodic storing timer.
  change_and_check<int64_t>("storing_period", 2);
  change_and_check<std::string>("persistent.a_string", "General");
  std::this_thread::sleep_for(3s);
  reload_and_check<std::string>("persistent.a_string", "General", "General");

  // increase the period: the old timer is cancelled and the new one does not fire yet.
  change_and_check<int64_t>("storing_period", 30);
  change_and_check<std::string>("persistent.a_string", "Kenobi");
  std::this_thread::sleep_for(3s);
  reload_and_check<std::string>("persistent.a_string", "Kenobi", "General");

  // disable the timer again: nothing is stored anymore.
  change_and_check<int64_t>("storing_period", 2);
  std::this_thread::sleep_for(2s);
  change_and_check<int64_t>("storing_period", 0);
  change_and_check<std::string>("persistent.a_string", "Kenobi");
  std::this_thread::sleep_for(3s);
  reload_and_check<std::string>("persistent.a_string", "Kenobi", "General");
}

TEST_F(ParameterServerTest, allow_dynamic_typing_can_be_changed_dynamically)
{
  ASSERT_NO_FATAL_FAILURE(start_server(false, 0, false));

  expect_value<bool>("allow_dynamic_typing", false);
  EXPECT_FALSE(client_->modify_parameter("persistent.some_int", "mutated"));

  // dynamically enable allow_dynamic_typing: the type change now succeeds,
  // even though the parameter was declared before the change.
  change_and_check<bool>("allow_dynamic_typing", true);
  change_and_check<std::string>("persistent.some_int", "mutated");
  change_and_check<int64_t>("persistent.some_int", 1);

  // dynamically disable allow_dynamic_typing: the type change is rejected again.
  change_and_check<bool>("allow_dynamic_typing", false);
  EXPECT_FALSE(client_->modify_parameter("persistent.some_int", "mutated"));
  expect_value<int64_t>("persistent.some_int", 1);
}

TEST_F(ParameterServerTest, invalid_server_parameter_values_are_rejected)
{
  ASSERT_NO_FATAL_FAILURE(start_server(false, 0, false));

  EXPECT_FALSE(client_->modify_parameter<int64_t>("storing_period", -1));
  // the wall timer period in nanoseconds must not overflow
  EXPECT_FALSE(
    client_->modify_parameter<int64_t>(
      "storing_period", std::numeric_limits<int64_t>::max() / 1000000000LL + 1));
  EXPECT_FALSE(client_->modify_parameter("storing_period", "invalid"));
  EXPECT_FALSE(client_->modify_parameter<int64_t>("must_save_on_update", 1));
  EXPECT_FALSE(client_->modify_parameter<int64_t>("allow_dynamic_typing", 1));
  // the server configuration parameters cannot be deleted.
  EXPECT_FALSE(client_->modify_parameter("storing_period", rclcpp::ParameterValue()));

  // rejected requests must not change the server configuration.
  expect_value<int64_t>("storing_period", 0);
  expect_value<bool>("must_save_on_update", false);
  expect_value<bool>("allow_dynamic_typing", false);
}

TEST_F(ParameterServerTest, server_parameters_are_declared_without_allow_override)
{
  // automatically_declare_parameters_from_overrides disabled:
  // the server configuration parameters must still be declared and reconfigurable.
  ASSERT_NO_FATAL_FAILURE(start_server(false, 0, true, false));

  expect_value<int64_t>("storing_period", 0);
  expect_value<bool>("allow_dynamic_typing", false);
  expect_value<bool>("must_save_on_update", true);

  change_and_check<bool>("must_save_on_update", false);
  change_and_check<bool>("allow_dynamic_typing", true);
  change_and_check<std::string>("persistent.some_int", "mutated");
  EXPECT_FALSE(client_->modify_parameter<int64_t>("storing_period", -1));
  change_and_check<int64_t>("storing_period", 2);
  expect_value<int64_t>("storing_period", 2);
}

TEST_F(ParameterServerTest, missing_yaml_file_is_tolerated)
{
  std::filesystem::remove(yaml_file_);
  ASSERT_NO_FATAL_FAILURE(start_server());

  expect_not_set("some_int");
  expect_not_set("persistent.some_int");

  // the file is created once a persistent parameter is stored.
  change_and_check<int64_t>("persistent.created", 5);
  stop_server();
  ASSERT_TRUE(std::filesystem::exists(yaml_file_));

  ASSERT_NO_FATAL_FAILURE(start_server());
  expect_value<int64_t>("persistent.created", 5);
}

TEST_F(ParameterServerTest, yaml_file_keyed_by_node_name)
{
  write_file(
    yaml_file_,
    "parameter_server:\n"
    "    ros__parameters:\n"
    "        persistent:\n"
    "            value: 1\n");
  ASSERT_NO_FATAL_FAILURE(start_server());

  expect_value<int64_t>("persistent.value", 1);
  change_and_check<int64_t>("persistent.value", 2);
  change_and_check<std::string>("persistent.added", "added");

  ASSERT_NO_FATAL_FAILURE(restart_server());
  expect_value<int64_t>("persistent.value", 2);
  expect_value<std::string>("persistent.added", "added");

  YAML::Node persistent = persistent_node_in_file("parameter_server");
  EXPECT_EQ(persistent["value"].as<int64_t>(), 2);
  EXPECT_EQ(persistent["added"].as<std::string>(), "added");
}

TEST_F(ParameterServerTest, all_parameter_types_survive_restart)
{
  ASSERT_NO_FATAL_FAILURE(start_server());

  const std::vector<bool> bools = {true, false, true};
  const std::vector<int64_t> ints = {-1, 0, 42};
  const std::vector<double> doubles = {-1.5, 0.0, 2.5};
  const std::vector<std::string> strings = {"a", "b c", "d"};
  const std::vector<uint8_t> bytes = {1, 2, 3};

  change_and_check<bool>("persistent.types.a_bool", true);
  change_and_check<int64_t>("persistent.types.an_int", -7);
  change_and_check<double>("persistent.types.a_double", -0.25);
  change_and_check<std::string>("persistent.types.a_string", "text");
  change_and_check("persistent.types.bools", bools);
  change_and_check("persistent.types.ints", ints);
  change_and_check("persistent.types.doubles", doubles);
  change_and_check("persistent.types.strings", strings);
  change_and_check("persistent.types.bytes", bytes);

  ASSERT_NO_FATAL_FAILURE(restart_server());

  expect_value<bool>("persistent.types.a_bool", true);
  expect_value<int64_t>("persistent.types.an_int", -7);
  expect_value<double>("persistent.types.a_double", -0.25);
  expect_value<std::string>("persistent.types.a_string", "text");
  expect_value("persistent.types.bools", bools);
  expect_value("persistent.types.ints", ints);
  expect_value("persistent.types.doubles", doubles);
  expect_value("persistent.types.strings", strings);
  // byte arrays are not supported by the yaml parameter parser,
  // so they are stored and loaded back as integer arrays.
  expect_value<std::vector<int64_t>>("persistent.types.bytes", {1, 2, 3});

  YAML::Node types = persistent_node_in_file()["types"];
  EXPECT_EQ(types["a_bool"].as<bool>(), true);
  EXPECT_EQ(types["an_int"].as<int64_t>(), -7);
  EXPECT_EQ(types["a_double"].as<std::string>(), "-0.25");
  EXPECT_EQ(types["a_string"].as<std::string>(), "text");
  EXPECT_EQ(types["bools"].as<std::vector<bool>>(), bools);
  EXPECT_EQ(types["ints"].as<std::vector<int64_t>>(), ints);
  EXPECT_EQ(types["strings"].as<std::vector<std::string>>(), strings);
  EXPECT_EQ(types["bytes"].as<std::vector<int64_t>>(), std::vector<int64_t>({1, 2, 3}));
}

TEST_F(ParameterServerTest, empty_arrays_are_not_stored)
{
  ASSERT_NO_FATAL_FAILURE(start_server());

  change_and_check("persistent.empty_ints", std::vector<int64_t>{});
  change_and_check("persistent.empty_strings", std::vector<std::string>{});
  change_and_check("persistent.empty_doubles", std::vector<double>{});

  ASSERT_NO_FATAL_FAILURE(restart_server());

  expect_not_set("persistent.empty_ints");
  expect_not_set("persistent.empty_strings");
  expect_not_set("persistent.empty_doubles");
  YAML::Node persistent = persistent_node_in_file();
  EXPECT_FALSE(persistent["empty_ints"]);
  EXPECT_FALSE(persistent["empty_strings"]);
  EXPECT_FALSE(persistent["empty_doubles"]);
}

TEST_F(ParameterServerTest, only_persistent_prefixed_parameters_are_stored)
{
  ASSERT_NO_FATAL_FAILURE(start_server());

  change_and_check<int64_t>("persistent_without_dot", 1);
  change_and_check<int64_t>("not.persistent.value", 2);
  change_and_check<int64_t>("persistent.nested.deep.value", 3);
  change_and_check<int64_t>("persistent.some_lists.some_integers.extra", 4);

  ASSERT_NO_FATAL_FAILURE(restart_server());

  expect_not_set("persistent_without_dot");
  expect_not_set("not.persistent.value");
  expect_value<int64_t>("persistent.nested.deep.value", 3);

  YAML::Node ros_parameters = YAML::LoadFile(yaml_file_)["/**"]["ros__parameters"];
  EXPECT_FALSE(ros_parameters["persistent_without_dot"]);
  EXPECT_FALSE(ros_parameters["not"]);
  EXPECT_EQ(ros_parameters["persistent"]["nested"]["deep"]["value"].as<int64_t>(), 3);
}

TEST_F(ParameterServerTest, persistent_parameter_can_be_deleted)
{
  ASSERT_NO_FATAL_FAILURE(start_server());

  // deleting changes the type to NOT_SET, which is rejected without dynamic typing.
  EXPECT_FALSE(client_->modify_parameter("persistent.a_string", rclcpp::ParameterValue()));
  expect_value<std::string>("persistent.a_string", "Hello world");

  ASSERT_NO_FATAL_FAILURE(restart_server(true));
  change_and_check<std::string>("persistent.new_string", "to be deleted");
  ASSERT_TRUE(client_->modify_parameter("persistent.a_string", rclcpp::ParameterValue()));
  ASSERT_TRUE(client_->modify_parameter("persistent.new_string", rclcpp::ParameterValue()));
  expect_not_set("persistent.a_string");
  expect_not_set("persistent.new_string");

  // the deleted parameters are removed from the file on restart.
  ASSERT_NO_FATAL_FAILURE(restart_server(true));
  expect_not_set("persistent.a_string");
  expect_not_set("persistent.new_string");
  expect_value<int64_t>("persistent.some_int", 1);
  YAML::Node persistent = persistent_node_in_file();
  EXPECT_FALSE(persistent["a_string"]);
  EXPECT_FALSE(persistent["new_string"]);
  EXPECT_EQ(persistent["some_int"].as<int64_t>(), 1);
}

TEST_F(ParameterServerTest, special_characters_in_strings_survive_restart)
{
  ASSERT_NO_FATAL_FAILURE(start_server());

  const std::string quoted = "it's \"quoted\" #not a comment: yes, [not, a, list] {}";
  const std::string unicode = "こんにちは";
  change_and_check<std::string>("persistent.quoted", quoted);
  change_and_check<std::string>("persistent.unicode", unicode);
  change_and_check<std::string>("persistent.empty", "");
  change_and_check<std::string>("persistent.looks_like_number", "42");
  change_and_check<std::string>("persistent.looks_like_bool", "true");

  ASSERT_NO_FATAL_FAILURE(restart_server());

  expect_value<std::string>("persistent.quoted", quoted);
  expect_value<std::string>("persistent.unicode", unicode);
  expect_value<std::string>("persistent.empty", "");
  expect_value<std::string>("persistent.looks_like_number", "42");
  expect_value<std::string>("persistent.looks_like_bool", "true");
}

TEST_F(ParameterServerTest, yaml_file_overrides_parameter_overrides)
{
  // the persistent file wins over the parameter overrides given on the command line,
  // regardless of automatically_declare_parameters_from_overrides.
  extra_overrides_.emplace_back("persistent.a_string", "FromOverride");
  extra_overrides_.emplace_back("from_override", 7);

  ASSERT_NO_FATAL_FAILURE(start_server(false, 0, false, true));
  expect_value<std::string>("persistent.a_string", "Hello world");
  expect_value<int64_t>("from_override", 7);

  ASSERT_NO_FATAL_FAILURE(restart_server(false, 0, false, false));
  expect_value<std::string>("persistent.a_string", "Hello world");
  expect_not_set("from_override");
}

TEST_F(ParameterServerTest, namespaced_server_uses_namespaced_yaml_section)
{
  server_namespace_ = "/ns";
  write_file(
    yaml_file_,
    "/ns:\n"
    "    parameter_server:\n"
    "        ros__parameters:\n"
    "            persistent:\n"
    "                value: 1\n");
  ASSERT_NO_FATAL_FAILURE(start_server());
  EXPECT_STREQ(server_->get_fully_qualified_name(), "/ns/parameter_server");

  expect_value<int64_t>("persistent.value", 1);
  change_and_check<int64_t>("persistent.value", 2);

  ASSERT_NO_FATAL_FAILURE(restart_server());
  expect_value<int64_t>("persistent.value", 2);
  YAML::Node config = YAML::LoadFile(yaml_file_);
  EXPECT_EQ(
    config["/ns"]["parameter_server"]["ros__parameters"]["persistent"]["value"].as<int64_t>(), 2);
}

TEST_F(ParameterServerTest, yaml_file_keyed_by_absolute_node_name)
{
  write_file(
    yaml_file_,
    "/parameter_server:\n"
    "    ros__parameters:\n"
    "        persistent:\n"
    "            value: 1\n");
  ASSERT_NO_FATAL_FAILURE(start_server());

  expect_value<int64_t>("persistent.value", 1);
  change_and_check<int64_t>("persistent.value", 2);

  ASSERT_NO_FATAL_FAILURE(restart_server());
  expect_value<int64_t>("persistent.value", 2);
  EXPECT_EQ(persistent_node_in_file("/parameter_server")["value"].as<int64_t>(), 2);
}

TEST_F(ParameterServerTest, save_and_reload_service_error_handling)
{
  ASSERT_NO_FATAL_FAILURE(start_server());

  // saving without any persistent change keeps the content of the file.
  ASSERT_TRUE(client_->trigger_save());
  YAML::Node ros_parameters = YAML::LoadFile(yaml_file_)["/**"]["ros__parameters"];
  EXPECT_EQ(ros_parameters["a_string"].as<std::string>(), "Hello world");
  EXPECT_EQ(ros_parameters["persistent"]["a_string"].as<std::string>(), "Hello world");
  EXPECT_EQ(ros_parameters["persistent"]["some_int"].as<int64_t>(), 1);
  EXPECT_DOUBLE_EQ(ros_parameters["persistent"]["pi"].as<double>(), 3.14);
  EXPECT_EQ(
    ros_parameters["persistent"]["some_lists"]["some_integers"].as<std::vector<int64_t>>(),
    std::vector<int64_t>({1, 2, 3, 4}));

  // reloading a missing file is tolerated and keeps the current values.
  change_and_check<std::string>("persistent.a_string", "Changed");
  std::filesystem::remove(yaml_file_);
  ASSERT_TRUE(client_->reload_params());
  expect_value<std::string>("persistent.a_string", "Changed");

  // reloading an invalid file fails and keeps the current values.
  write_file(yaml_file_, "first:\n    a: 1\nsecond:\n    b: 2\n");
  EXPECT_FALSE(client_->reload_params());
  expect_value<std::string>("persistent.a_string", "Changed");

  // saving recreates a valid file from the current persistent parameters.
  write_file(yaml_file_, kDefaultYaml);
  ASSERT_TRUE(client_->trigger_save());
  EXPECT_EQ(persistent_node_in_file()["a_string"].as<std::string>(), "Changed");
}

TEST_F(ParameterServerTest, periodic_storing_with_initial_storing_period)
{
  ASSERT_NO_FATAL_FAILURE(start_server(false, 2, false));
  expect_value<int64_t>("storing_period", 2);

  change_and_check<std::string>("persistent.a_string", "Timer");
  std::this_thread::sleep_for(3s);
  EXPECT_EQ(persistent_node_in_file()["a_string"].as<std::string>(), "Timer");
  reload_and_check<std::string>("persistent.a_string", "Discarded", "Timer");
}

TEST_F(ParameterServerTest, invalid_initial_storing_period_is_coerced_to_zero)
{
  ASSERT_NO_FATAL_FAILURE(start_server(false, -5, false));
  expect_value<int64_t>("storing_period", 0);

  // no periodic storing takes place.
  change_and_check<std::string>("persistent.a_string", "NotStored");
  std::this_thread::sleep_for(1s);
  reload_and_check<std::string>("persistent.a_string", "NotStored", "Hello world");
}

TEST_F(ParameterServerTest, invalid_yaml_file_throws)
{
  rclcpp::NodeOptions options;

  // more than one top level key is not a valid parameter file.
  write_file(yaml_file_, "first:\n    ros__parameters:\n        a: 1\nsecond:\n    b: 2\n");
  EXPECT_THROW(
    std::make_shared<ParameterServer>("parameter_server", options, yaml_file_),
    std::runtime_error);

  // node name must match or be "/**".
  write_file(yaml_file_, "other_node:\n    ros__parameters:\n        a: 1\n");
  EXPECT_THROW(
    std::make_shared<ParameterServer>("parameter_server", options, yaml_file_),
    std::runtime_error);

  // a map key must not contain '.'.
  write_file(yaml_file_, "/**:\n    ros__parameters:\n        a.b:\n            c: 1\n");
  EXPECT_THROW(
    std::make_shared<ParameterServer>("parameter_server", options, yaml_file_),
    std::runtime_error);
}
