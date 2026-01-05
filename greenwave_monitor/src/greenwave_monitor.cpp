// SPDX-FileCopyrightText: NVIDIA CORPORATION & AFFILIATES
// Copyright (c) 2025-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
// http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.
//
// SPDX-License-Identifier: Apache-2.0

#include "greenwave_monitor.hpp"

#include <cstring>

#include "rcl_interfaces/msg/parameter_descriptor.hpp"
#include "rcl_interfaces/srv/list_parameters.hpp"
#include "rosidl_typesupport_introspection_cpp/message_introspection.hpp"

using namespace std::chrono_literals;

namespace
{
constexpr const char * kTopicParamPrefix = "topics.";
constexpr const char * kFreqSuffix = ".expected_frequency";
constexpr const char * kTolSuffix = ".tolerance";
constexpr double kDefaultTolerancePercent = 5.0;

std::string make_freq_param_name(const std::string & topic_name)
{
  return std::string(kTopicParamPrefix) + topic_name + kFreqSuffix;
}

std::string make_tol_param_name(const std::string & topic_name)
{
  return std::string(kTopicParamPrefix) + topic_name + kTolSuffix;
}

enum class TopicParamField { kNone, kFrequency, kTolerance };

struct TopicParamInfo
{
  std::string topic_name;
  TopicParamField field = TopicParamField::kNone;
};

// Parse a parameter name like "topics./my_topic.expected_frequency" into topic name and field type
TopicParamInfo parse_topic_param_name(const std::string & param_name)
{
  TopicParamInfo info;

  if (param_name.rfind(kTopicParamPrefix, 0) != 0) {
    return info;
  }

  std::string topic_and_field = param_name.substr(strlen(kTopicParamPrefix));

  const size_t freq_suffix_len = strlen(kFreqSuffix);
  const size_t tol_suffix_len = strlen(kTolSuffix);
  const size_t len = topic_and_field.length();

  bool is_freq = len > freq_suffix_len &&
    topic_and_field.rfind(kFreqSuffix) == len - freq_suffix_len;
  bool is_tol = len > tol_suffix_len &&
    topic_and_field.rfind(kTolSuffix) == len - tol_suffix_len;

  if (is_freq) {
    info.topic_name = topic_and_field.substr(0, len - freq_suffix_len);
    info.field = TopicParamField::kFrequency;
  } else if (is_tol) {
    info.topic_name = topic_and_field.substr(0, len - tol_suffix_len);
    info.field = TopicParamField::kTolerance;
  }

  return info;
}

// Convert a parameter to double if it's a numeric type
std::optional<double> param_to_double(const rclcpp::Parameter & param)
{
  if (param.get_type() == rclcpp::ParameterType::PARAMETER_DOUBLE) {
    return param.as_double();
  } else if (param.get_type() == rclcpp::ParameterType::PARAMETER_INTEGER) {
    return static_cast<double>(param.as_int());
  }
  return std::nullopt;
}

}  // namespace

GreenwaveMonitor::GreenwaveMonitor(const rclcpp::NodeOptions & options)
: Node("greenwave_monitor",
    rclcpp::NodeOptions(options)
    .allow_undeclared_parameters(true)
    .automatically_declare_parameters_from_overrides(true))
{
  RCLCPP_INFO(this->get_logger(), "Starting GreenwaveMonitorNode");

  // Get the topics parameter (declare only if not already declared from overrides)
  if (!this->has_parameter("topics")) {
    this->declare_parameter<std::vector<std::string>>("topics", {""});
  }
  auto topics = this->get_parameter("topics").as_string_array();

  message_diagnostics::MessageDiagnosticsConfig diagnostics_config;
  diagnostics_config.enable_all_topic_diagnostics = true;

  auto topic_names_and_types = this->get_topic_names_and_types();

  for (const auto & topic : topics) {
    if (!topic.empty()) {
      std::string message;
      add_topic(topic, message);
    }
  }

  // Register parameter callback for dynamic topic configuration
  param_callback_handle_ = this->add_on_set_parameters_callback(
    std::bind(&GreenwaveMonitor::on_parameter_change, this, std::placeholders::_1));

  // Subscribe to parameter events to handle parameter deletions
  param_event_subscription_ = this->create_subscription<rcl_interfaces::msg::ParameterEvent>(
    "/parameter_events", 10,
    std::bind(&GreenwaveMonitor::on_parameter_event, this, std::placeholders::_1));

  // Process any topic parameters passed at startup
  load_topic_parameters_from_overrides();

  timer_ = this->create_wall_timer(
    1s, std::bind(&GreenwaveMonitor::timer_callback, this));

  // Add service server
  manage_topic_service_ =
    this->create_service<greenwave_monitor_interfaces::srv::ManageTopic>(
    "~/manage_topic",
    std::bind(
      &GreenwaveMonitor::handle_manage_topic, this,
      std::placeholders::_1, std::placeholders::_2));

  set_expected_frequency_service_ =
    this->create_service<greenwave_monitor_interfaces::srv::SetExpectedFrequency>(
    "~/set_expected_frequency",
    std::bind(
      &GreenwaveMonitor::handle_set_expected_frequency, this,
      std::placeholders::_1, std::placeholders::_2));
}

std::optional<std::string> GreenwaveMonitor::find_topic_type_with_retry(
  const std::string & topic, const int max_retries, const int retry_period_s)
{
  for (int i = 0; i < max_retries; ++i) {
    auto topic_names_and_types = this->get_topic_names_and_types();
    auto it = topic_names_and_types.find(topic);
    if (it == topic_names_and_types.end() || it->second.empty()) {
      std::this_thread::sleep_for(std::chrono::seconds(retry_period_s));
      continue;
    } else {
      return it->second[0];
    }
  }
  return std::nullopt;
}

void GreenwaveMonitor::topic_callback(
  const std::shared_ptr<rclcpp::SerializedMessage> msg,
  const std::string & topic, const std::string & type)
{
  auto msg_timestamp = GetTimestampFromSerializedMessage(msg, type);
  message_diagnostics_[topic]->updateDiagnostics(msg_timestamp.time_since_epoch().count());
}

void GreenwaveMonitor::timer_callback()
{
  RCLCPP_INFO(this->get_logger(), "====================================================");
  if (message_diagnostics_.empty()) {
    RCLCPP_INFO(this->get_logger(), "No topics to monitor");
  }
  for (auto & [topic, diagnostics] : message_diagnostics_) {
    diagnostics->publishDiagnostics();
    RCLCPP_INFO(
      this->get_logger(), "Frame rate for topic %s: %.2f hz",
      topic.c_str(), diagnostics->getFrameRateNode());
    RCLCPP_INFO(
      this->get_logger(), "Latency for topic %s: %.2f ms",
      topic.c_str(), diagnostics->getLatency());
  }
  RCLCPP_INFO(this->get_logger(), "====================================================");
}

void GreenwaveMonitor::handle_manage_topic(
  const std::shared_ptr<greenwave_monitor_interfaces::srv::ManageTopic::Request> request,
  std::shared_ptr<greenwave_monitor_interfaces::srv::ManageTopic::Response> response)
{
  if (request->add_topic) {
    response->success = add_topic(request->topic_name, response->message);
  } else {
    response->success = remove_topic(request->topic_name, response->message);
  }
}

void GreenwaveMonitor::handle_set_expected_frequency(
  const std::shared_ptr<greenwave_monitor_interfaces::srv::SetExpectedFrequency::Request> request,
  std::shared_ptr<greenwave_monitor_interfaces::srv::SetExpectedFrequency::Response> response)
{
  auto it = message_diagnostics_.find(request->topic_name);

  if (it == message_diagnostics_.end()) {
    if (!request->add_topic_if_missing) {
      response->success = false;
      response->message = "Failed to find topic";
      return;
    }

    if (!add_topic(request->topic_name, response->message)) {
      response->success = false;
      return;
    }
    it = message_diagnostics_.find(request->topic_name);
  }

  message_diagnostics::MessageDiagnostics & msg_diagnostics_obj = *(it->second);

  if (request->clear_expected) {
    msg_diagnostics_obj.clearExpectedDt();
    undeclare_topic_parameters(request->topic_name);

    response->success = true;
    response->message = "Successfully cleared expected frequency for topic '" +
      request->topic_name + "'";
    return;
  }

  response->success = set_topic_expected_frequency(
    request->topic_name,
    request->expected_hz,
    request->tolerance_percent,
    false,  // topic already exists at this point
    response->message);
}

bool GreenwaveMonitor::set_topic_expected_frequency(
  const std::string & topic_name,
  double expected_hz,
  double tolerance_percent,
  bool add_topic_if_missing,
  std::string & message,
  bool update_parameters)
{
  if (expected_hz <= 0.0) {
    message = "Invalid expected frequency, must be set to a positive value";
    return false;
  }
  if (tolerance_percent < 0.0) {
    message = "Invalid tolerance, must be a non-negative percentage";
    return false;
  }

  auto it = message_diagnostics_.find(topic_name);

  if (it == message_diagnostics_.end()) {
    if (!add_topic_if_missing) {
      message = "Failed to find topic '" + topic_name + "'";
      return false;
    }

    if (!add_topic(topic_name, message)) {
      return false;
    }
    it = message_diagnostics_.find(topic_name);
  }

  message_diagnostics::MessageDiagnostics & msg_diagnostics_obj = *(it->second);
  msg_diagnostics_obj.setExpectedDt(expected_hz, tolerance_percent);

  // Sync parameters with the new values
  if (update_parameters) {
    updating_params_internally_ = true;
    try {
      declare_or_set_parameter(make_freq_param_name(topic_name), expected_hz);
      declare_or_set_parameter(make_tol_param_name(topic_name), tolerance_percent);
    } catch (const std::exception & e) {
      message = "Could not set parameters for topic '" + topic_name + "': " + e.what();
      updating_params_internally_ = false;
      return false;
    }
    updating_params_internally_ = false;
  }

  message = "Successfully set expected frequency for topic '" +
    topic_name + "' to " + std::to_string(expected_hz) +
    " hz with tolerance " + std::to_string(tolerance_percent) + "%";
  return true;
}

rcl_interfaces::msg::SetParametersResult GreenwaveMonitor::on_parameter_change(
  const std::vector<rclcpp::Parameter> & parameters)
{
  rcl_interfaces::msg::SetParametersResult result;
  result.successful = true;

  // Skip if updating from within the node (avoids redundant work and deadlock)
  if (updating_params_internally_) {
    return result;
  }

  // Collect validation errors and valid configs
  std::vector<std::string> errors;
  std::map<std::string, TopicConfig> incoming_configs;

  // Construct expected frequency and tolerance pairs from parameter changes
  for (const auto & param : parameters) {
    auto info = parse_topic_param_name(param.get_name());
    if (info.field == TopicParamField::kNone || info.topic_name.empty()) {
      continue;
    }

    // Allow PARAMETER_NOT_SET for parameter deletion
    if (param.get_type() == rclcpp::ParameterType::PARAMETER_NOT_SET) {
      continue;
    }

    auto value_opt = param_to_double(param);
    if (!value_opt.has_value()) {
      errors.push_back(param.get_name() + ": must be a numeric type (int or double)");
      continue;
    }

    double value = value_opt.value();
    TopicConfig & config = incoming_configs[info.topic_name];

    if (info.field == TopicParamField::kFrequency) {
      if (value <= 0.0) {
        errors.push_back(
          param.get_name() + ": Invalid frequency, must be positive");
        continue;
      }
      config.expected_frequency = value;
    } else {
      if (value < 0.0) {
        errors.push_back(
          param.get_name() + ": Invalid tolerance, must be non-negative");
        continue;
      }
      config.tolerance = value;
    }
  }

  // Iterate over incoming configs and set expected frequencies/tolerances
  for (const auto & [topic_name, incoming] : incoming_configs) {
    // Get expected frequency: prefer incoming, fall back to existing parameter
    double expected_freq = 0.0;
    if (incoming.expected_frequency.has_value()) {
      expected_freq = incoming.expected_frequency.value();
    } else {
      auto freq_opt = get_numeric_parameter(make_freq_param_name(topic_name));
      if (freq_opt.has_value()) {
        expected_freq = freq_opt.value();
      } else {
        // Tolerance set without frequency - nothing to apply yet
        continue;
      }
    }

    // Get tolerance: prefer incoming, then existing parameter, then default
    double tolerance = incoming.tolerance.value_or(
      get_numeric_parameter(make_tol_param_name(topic_name)).value_or(kDefaultTolerancePercent)
    );

    std::string message;
    bool success = set_topic_expected_frequency(
      topic_name,
      expected_freq,
      tolerance,
      true,
      message,
      false);  // don't update parameters - called from parameter change

    // Log warning if the topic is not up yet or an error occurs while trying to monitor it
    // Still errors if parameter is invalid value since that is redundantly checked earlier
    if (!success) {
      RCLCPP_WARN(
        this->get_logger(),
        "Could not apply monitoring config for topic '%s': %s",
        topic_name.c_str(), message.c_str());
    }
  }

  if (!errors.empty()) {
    result.successful = false;
    result.reason = "Invalid parameters: " + rcpputils::join(errors, ", ");
  }

  return result;
}

void GreenwaveMonitor::on_parameter_event(
  const rcl_interfaces::msg::ParameterEvent::SharedPtr msg)
{
  // Only process events from this node
  if (msg->node != this->get_fully_qualified_name()) {
    return;
  }

  for (const auto & param : msg->deleted_parameters) {
    auto info = parse_topic_param_name(param.name);
    if (info.field == TopicParamField::kNone || info.topic_name.empty()) {
      continue;
    }

    auto it = message_diagnostics_.find(info.topic_name);
    if (it == message_diagnostics_.end()) {
      continue;
    }

    if (info.field == TopicParamField::kFrequency) {
      it->second->clearExpectedDt();
      RCLCPP_DEBUG(
        this->get_logger(),
        "Cleared expected frequency for topic '%s' (parameter deleted)",
        info.topic_name.c_str());
    } else if (info.field == TopicParamField::kTolerance) {
      // Reset tolerance to default if frequency is still set
      auto freq_opt = get_numeric_parameter(make_freq_param_name(info.topic_name));
      if (freq_opt.has_value() && freq_opt.value() > 0) {
        it->second->setExpectedDt(freq_opt.value(), kDefaultTolerancePercent);
        RCLCPP_DEBUG(
          this->get_logger(),
          "Reset tolerance to default (%.1f%%) for topic '%s' (parameter deleted)",
          kDefaultTolerancePercent, info.topic_name.c_str());
      }
    }
  }
}

void GreenwaveMonitor::load_topic_parameters_from_overrides()
{
  // Parameters are automatically declared from overrides due to NodeOptions setting.
  // List all parameters and filter by prefix manually (list_parameters prefix matching
  // can be unreliable with deeply nested parameter names).
  auto all_params = this->list_parameters(
    {}, rcl_interfaces::srv::ListParameters::Request::DEPTH_RECURSIVE);

  // Build a local map of topic configs from startup parameters
  std::map<std::string, TopicConfig> startup_configs;

  // Construct expected frequency and tolerance pairs from startup parameters
  for (const auto & name : all_params.names) {
    auto info = parse_topic_param_name(name);
    if (info.field == TopicParamField::kNone || info.topic_name.empty()) {
      continue;
    }

    auto value_opt = get_numeric_parameter(name);
    if (!value_opt.has_value()) {
      continue;
    }

    double value = value_opt.value();
    TopicConfig & config = startup_configs[info.topic_name];

    if (info.field == TopicParamField::kFrequency) {
      config.expected_frequency = value;
    } else {
      config.tolerance = value;
    }
  }

  // Iterate over starting config and add topics/set expected frequencies
  for (const auto & [topic, config] : startup_configs) {
    // Topics will only be added if frequency is set
    if (config.expected_frequency.has_value()) {
      double tolerance = config.tolerance.value_or(kDefaultTolerancePercent);

      std::string message;
      bool success = set_topic_expected_frequency(
        topic,
        config.expected_frequency.value(),
        tolerance,
        true,   // add topic if missing - safe at startup
        message,
        false);  // don't update parameters

      if (!success) {
        RCLCPP_WARN(this->get_logger(), "%s", message.c_str());
      }
    }
  }
}

std::optional<double> GreenwaveMonitor::get_numeric_parameter(const std::string & param_name)
{
  if (!this->has_parameter(param_name)) {
    return std::nullopt;
  }
  return param_to_double(this->get_parameter(param_name));
}

void GreenwaveMonitor::try_undeclare_parameter(const std::string & param_name)
{
  try {
    if (this->has_parameter(param_name)) {
      this->undeclare_parameter(param_name);
    }
  } catch (const std::exception & e) {
    RCLCPP_WARN(
      this->get_logger(), "Could not undeclare %s: %s",
      param_name.c_str(), e.what());
  }
}

void GreenwaveMonitor::declare_or_set_parameter(const std::string & param_name, double value)
{
  if (!this->has_parameter(param_name)) {
    // Allow both integer and double types for numeric parameters
    rcl_interfaces::msg::ParameterDescriptor descriptor;
    descriptor.dynamic_typing = true;
    this->declare_parameter(param_name, value, descriptor);
  } else {
    this->set_parameter(rclcpp::Parameter(param_name, value));
  }
}

void GreenwaveMonitor::undeclare_topic_parameters(const std::string & topic_name)
{
  try_undeclare_parameter(make_freq_param_name(topic_name));
  try_undeclare_parameter(make_tol_param_name(topic_name));
}

bool GreenwaveMonitor::has_header_from_type(const std::string & type_name)
{
  // We use a cache to avoid repeated lookups for the same message type.
  // ex. {sensor_msgs/msg/Image : true, std_msgs/msg/String : false}
  static std::unordered_map<std::string, bool> type_has_header_cache;

  static std::mutex has_header_cache_mutex;
  std::lock_guard<std::mutex> lock(has_header_cache_mutex);

  if (type_has_header_cache.find(type_name) != type_has_header_cache.end()) {
    return type_has_header_cache[type_name];
  }

  // rosidl typesupport API is unstable across ROS distributions, so we use this
  // map as a more robust way to determine if a message type has a header
  static const std::unordered_map<std::string, bool> known_header_types = {
    // sensor_msgs
    {"sensor_msgs/msg/Image", true},
    {"sensor_msgs/msg/CompressedImage", true},
    {"sensor_msgs/msg/CameraInfo", true},
    {"sensor_msgs/msg/PointCloud2", true},
    {"sensor_msgs/msg/LaserScan", true},
    {"sensor_msgs/msg/Imu", true},
    {"sensor_msgs/msg/NavSatFix", true},
    {"sensor_msgs/msg/MagneticField", true},
    {"sensor_msgs/msg/FluidPressure", true},
    {"sensor_msgs/msg/Illuminance", true},
    {"sensor_msgs/msg/RelativeHumidity", true},
    {"sensor_msgs/msg/Temperature", true},
    {"sensor_msgs/msg/Range", true},
    {"sensor_msgs/msg/PointCloud", true},

    // geometry_msgs
    {"geometry_msgs/msg/PoseStamped", true},
    {"geometry_msgs/msg/TwistStamped", true},
    {"geometry_msgs/msg/AccelStamped", true},
    {"geometry_msgs/msg/Vector3Stamped", true},
    {"geometry_msgs/msg/PointStamped", true},
    {"geometry_msgs/msg/QuaternionStamped", true},
    {"geometry_msgs/msg/TransformStamped", true},
    {"geometry_msgs/msg/WrenchStamped", true},

    // nav_msgs
    {"nav_msgs/msg/OccupancyGrid", true},
    {"nav_msgs/msg/GridCells", true},
    {"nav_msgs/msg/Path", true},
    {"nav_msgs/msg/Odometry", true},

    // visualization_msgs
    {"visualization_msgs/msg/Marker", true},
    {"visualization_msgs/msg/MarkerArray", true},
    {"visualization_msgs/msg/InteractiveMarker", true},

    // std_msgs (no headers)
    {"std_msgs/msg/String", false},
    {"std_msgs/msg/Int32", false},
    {"std_msgs/msg/Float64", false},
    {"std_msgs/msg/Bool", false},
    {"std_msgs/msg/Empty", false},
    {"std_msgs/msg/Header", false},  // Header itself doesn't have a header

    // Common message types without headers
    {"geometry_msgs/msg/Twist", false},
    {"geometry_msgs/msg/Pose", false},
    {"geometry_msgs/msg/Point", false},
    {"geometry_msgs/msg/Vector3", false},
    {"geometry_msgs/msg/Quaternion", false}
  };

  auto it = known_header_types.find(type_name);
  bool has_header = (it != known_header_types.end()) ? it->second : false;

  type_has_header_cache[type_name] = has_header;

  // Fallback of no header in case of unknown type, log for reference
  if (it == known_header_types.end()) {
    RCLCPP_WARN_ONCE(
      this->get_logger(),
      "Unknown message type '%s' - assuming no header. Consider adding to registry.",
      type_name.c_str());
  }

  return has_header;
}

bool GreenwaveMonitor::add_topic(const std::string & topic, std::string & message)
{
  // Check if topic already exists
  if (message_diagnostics_.find(topic) != message_diagnostics_.end()) {
    message = "Topic already being monitored";
    return false;
  }

  RCLCPP_INFO(this->get_logger(), "Adding subscription for topic '%s'", topic.c_str());

  const int max_retries = 5;
  const int retry_period_s = 1;
  auto maybe_type = find_topic_type_with_retry(topic, max_retries, retry_period_s);
  if (!maybe_type.has_value()) {
    RCLCPP_ERROR(this->get_logger(), "Failed to find type for topic '%s'", topic.c_str());
    message = "Failed to find type for topic";
    return false;
  }

  const std::string type = maybe_type.value();
  auto sub = this->create_generic_subscription(
    topic,
    type,
    rclcpp::QoS(
      rclcpp::KeepLast(10), rmw_qos_profile_sensor_data),
    [this, topic, type](std::shared_ptr<rclcpp::SerializedMessage> msg) {
      this->topic_callback(msg, topic, type);
    });

  message_diagnostics::MessageDiagnosticsConfig diagnostics_config;
  diagnostics_config.enable_all_topic_diagnostics = true;

  subscriptions_.push_back(sub);
  message_diagnostics_.emplace(
    topic,
    std::make_unique<message_diagnostics::MessageDiagnostics>(*this, topic, diagnostics_config));

  // If parameters are set, use them to set the expected frequency and tolerance
  auto freq_opt = get_numeric_parameter(make_freq_param_name(topic));
  auto tol_opt = get_numeric_parameter(make_tol_param_name(topic));
  double tolerance = tol_opt.value_or(kDefaultTolerancePercent);
  if (freq_opt.has_value() && tolerance >= 0.0 && freq_opt.value() > 0.0) {
    message_diagnostics_[topic]->setExpectedDt(freq_opt.value(), tolerance);
  }

  message = "Successfully added topic";
  return true;
}

bool GreenwaveMonitor::remove_topic(const std::string & topic, std::string & message)
{
  auto diag_it = message_diagnostics_.find(topic);
  if (diag_it == message_diagnostics_.end()) {
    message = "Topic not found";
    return false;
  }

  // Find and remove the subscription
  auto sub_it = std::find_if(
    subscriptions_.begin(), subscriptions_.end(),
    [&topic](const auto & sub) {
      return sub->get_topic_name() == topic;
    });

  if (sub_it != subscriptions_.end()) {
    subscriptions_.erase(sub_it);
  }

  message_diagnostics_.erase(diag_it);

  // Clear any associated parameters
  undeclare_topic_parameters(topic);

  message = "Successfully removed topic";
  return true;
}

// From ros2_benchmark monitor_node.cpp
// This assumes the message has a std_msgs header as the first
std::chrono::time_point<std::chrono::system_clock>
GreenwaveMonitor::GetTimestampFromSerializedMessage(
  std::shared_ptr<rclcpp::SerializedMessage> serialized_message_ptr,
  const std::string & type)
{
  if (!has_header_from_type(type)) {
    return std::chrono::time_point<std::chrono::system_clock>();  // timestamp 0 as fallback
  }

  int32_t timestamp_sec;
  uint8_t * sec_byte_ptr = reinterpret_cast<uint8_t *>(&timestamp_sec);
  *(sec_byte_ptr + 0) = serialized_message_ptr->get_rcl_serialized_message().buffer[4];
  *(sec_byte_ptr + 1) = serialized_message_ptr->get_rcl_serialized_message().buffer[5];
  *(sec_byte_ptr + 2) = serialized_message_ptr->get_rcl_serialized_message().buffer[6];
  *(sec_byte_ptr + 3) = serialized_message_ptr->get_rcl_serialized_message().buffer[7];

  uint32_t timestamp_nanosec;
  uint8_t * ns_byte_ptr = reinterpret_cast<uint8_t *>(&timestamp_nanosec);
  *(ns_byte_ptr + 0) = serialized_message_ptr->get_rcl_serialized_message().buffer[8];
  *(ns_byte_ptr + 1) = serialized_message_ptr->get_rcl_serialized_message().buffer[9];
  *(ns_byte_ptr + 2) = serialized_message_ptr->get_rcl_serialized_message().buffer[10];
  *(ns_byte_ptr + 3) = serialized_message_ptr->get_rcl_serialized_message().buffer[11];

  std::chrono::time_point<std::chrono::system_clock> timestamp(
    std::chrono::seconds(timestamp_sec) + std::chrono::nanoseconds(timestamp_nanosec));
  return timestamp;
}
