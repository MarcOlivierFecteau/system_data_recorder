// Copyright 2021 Open Source Robotics Foundation, Inc.
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

#include "sdr/sdr_component.hpp"

#include <chrono>
#include <ctime>
#include <iomanip>
#include <sstream>
#include <unistd.h>

#include "lifecycle_msgs/msg/state.hpp"
// In Humble, Rosbag2QoS lives in rosbag2_transport. In Jazzy+, it moved to
// rosbag2_storage.
#ifdef ROS_DISTRO_HUMBLE
#include "rosbag2_transport/qos.hpp"
#else
#include "rosbag2_storage/qos.hpp"
#endif

namespace sdr
{

SystemDataRecorder::SystemDataRecorder(
  const std::string & node_name,
  const rclcpp::NodeOptions & options)
: rclcpp_lifecycle::LifecycleNode(node_name, options)
{
  // Declare all parameters with defaults.
  // Actual values are read and validated in on_configure().
  declare_parameter("bags_dir", "");
  declare_parameter("session_name", "");
  declare_parameter("storage_type", "mcap");
  declare_parameter("max_file_size", int64_t(100));
  declare_parameter("autostart", false);
  declare_parameter("copy_bags", false);
  declare_parameter("copy_dir", "");
  declare_parameter("storage_preset_profile", "fastwrite");
  // Each entry encodes a topic/type pair as "topic_name:message_type".
  declare_parameter(
    "topics_and_types",
    std::vector<std::string>{
      "/parameter_events:rcl_interfaces/msg/ParameterEvent",
      "/rosout:rcl_interfaces/msg/Log"});
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
SystemDataRecorder::on_configure(const rclcpp_lifecycle::State & /* state */)
{
  RCLCPP_INFO(get_logger(), "Preparing to begin recording");

  // --- bags_dir ---
  auto bags_dir_str = get_parameter("bags_dir").as_string();
  if (bags_dir_str.empty()) {
    RCLCPP_ERROR(get_logger(), "Parameter 'bags_dir' must be specified");
    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::FAILURE;
  }
  std::filesystem::path bags_dir(bags_dir_str);
  if (!ensure_directory_writable(bags_dir)) {
    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::FAILURE;
  }

  // --- session_name ---
  auto session_name = get_parameter("session_name").as_string();
  if (session_name.empty()) {
    auto now = std::chrono::system_clock::now();
    std::time_t t = std::chrono::system_clock::to_time_t(now);
    std::tm tm_buf{};
    localtime_r(&t, &tm_buf);
    std::ostringstream oss;
    oss << std::put_time(&tm_buf, "%Y%m%d_%H%M%S");
    session_name = oss.str();
  }

  // --- storage_type ---
  auto storage_type = get_parameter("storage_type").as_string();
  if (storage_type != "mcap" && storage_type != "sqlite3") {
    RCLCPP_ERROR(
      get_logger(),
      "Parameter 'storage_type' must be \"mcap\" or \"sqlite3\", got: \"%s\"",
      storage_type.c_str());
    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::FAILURE;
  }
  const std::string bag_ext = (storage_type == "mcap") ? ".mcap" : ".db3";

  // --- max_file_size (MB->B) ---
  int64_t max_file_size_bytes =
      get_parameter("max_file_size").as_int() * 1024 * 1024;

  // --- copy_bags / copy_dir ---
  copy_bags_ = get_parameter("copy_bags").as_bool();
  std::filesystem::path copy_dir;
  if (copy_bags_) {
    auto copy_dir_str = get_parameter("copy_dir").as_string();
    if (copy_dir_str.empty()) {
      RCLCPP_ERROR(
        get_logger(),
        "Parameter 'copy_dir' must be specified when 'copy_bags' is true");
      return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::FAILURE;
    }
    copy_dir = std::filesystem::path(copy_dir_str);
    if (!ensure_directory_writable(copy_dir)) {
      return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::FAILURE;
    }
  }

  // --- topics_and_types: each entry is "topic_name:message_type" ---
  auto topics_and_types_param = get_parameter("topics_and_types").as_string_array();
  topics_and_types_.clear();
  for (const auto & entry : topics_and_types_param) {
    const auto sep = entry.find(':');
    if (sep == std::string::npos || sep == 0 || sep == entry.size() - 1) {
      RCLCPP_ERROR(
        get_logger(),
        "Invalid 'topics_and_types' entry \"%s\": expected format is \"topic_name:message_type\"",
        entry.c_str());
      return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::FAILURE;
    }
    topics_and_types_[entry.substr(0, sep)] = entry.substr(sep + 1);
  }

  // --- Build bag path: bags_dir/session_name ---
  // If the path already exists, append an incremental counter (_1, _2, …) until a
  // free slot is found, then update session_name so all subsequent paths stay consistent.
  source_directory_ = bags_dir / session_name;
  if (std::filesystem::exists(source_directory_)) {
    const std::string base_session_name = session_name;
    int counter = 1;
    do {
      session_name = base_session_name + "_" + std::to_string(counter++);
      source_directory_ = bags_dir / session_name;
    } while (std::filesystem::exists(source_directory_));
    RCLCPP_WARN(
      get_logger(),
      "Session path '%s' already exists; using '%s' instead",
      (bags_dir / base_session_name).c_str(),
      source_directory_.c_str());
  }

  // --- Storage options ---
  storage_options_.uri = source_directory_.string();
  storage_options_.storage_id = storage_type;
  storage_options_.max_bagfile_size =
      static_cast<uint64_t>(max_file_size_bytes);
  // Write cache reduces disk I/O pressure; set to 0 to disable
  storage_options_.max_cache_size = 100ULL * 1024ULL * 1024ULL;
  storage_options_.storage_preset_profile =
      get_parameter("storage_preset_profile").as_string();

  // Track which file is currently being written (rosbag2 names files _0, _1, …)
  last_bag_file_ = (source_directory_ / (session_name + "_0" + bag_ext)).string();

  // --- Optional copy thread ---
  if (copy_bags_) {
    destination_directory_ = copy_dir / session_name;
    RCLCPP_INFO(get_logger(), "Copying bag files to %s", destination_directory_.c_str());
    try {
      if (!create_copy_destination()) {
        return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::FAILURE;
      }
    } catch (const std::filesystem::filesystem_error & ex) {
      RCLCPP_ERROR(
        get_logger(),
        "Could not create copy destination directory: %s", ex.what());
      return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::FAILURE;
    }
    copy_thread_ = std::make_shared<std::thread>([this] { copy_thread_main(); });
    notify_state_change(SdrStateChange::PAUSED);
  }

  // --- Writer ---
  writer_ = std::make_shared<rosbag2_cpp::Writer>(
    std::make_unique<rosbag2_cpp::writers::SequentialWriter>());

  rosbag2_cpp::bag_events::WriterEventCallbacks callbacks;
  callbacks.write_split_callback =
    [this](rosbag2_cpp::bag_events::BagSplitInfo & info) {
      // Keep track of the file currently being written
      last_bag_file_ = info.opened_file;
      // Enqueue the just-closed file for copying (if enabled)
      if (copy_bags_) {
        notify_new_file_to_copy(info.closed_file);
      }
    };
  writer_->add_event_callbacks(callbacks);
  writer_->open(
    storage_options_,
    {rmw_get_serialization_format(), rmw_get_serialization_format()});

  subscribe_to_topics();

  cleaned_up_ = false;
  return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
SystemDataRecorder::on_activate(const rclcpp_lifecycle::State & /* state */)
{
  RCLCPP_INFO(get_logger(), "Starting recording");
  if (copy_bags_) {
    notify_state_change(SdrStateChange::RECORDING);
  }
  return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
SystemDataRecorder::on_deactivate(const rclcpp_lifecycle::State & /* state */)
{
  RCLCPP_INFO(get_logger(), "Pausing recording");
  if (copy_bags_) {
    notify_state_change(SdrStateChange::PAUSED);
  }
  return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
SystemDataRecorder::on_cleanup(const rclcpp_lifecycle::State & /* state */)
{
  RCLCPP_INFO(get_logger(), "Stopping and finalising recording");
  cleaned_up_ = true;

  unsubscribe_from_topics();
  writer_.reset();

  if (copy_bags_) {
    notify_new_file_to_copy(last_bag_file_);
    notify_new_file_to_copy(source_directory_ / "metadata.yaml");
    notify_state_change(SdrStateChange::FINISHED);
    copy_thread_->join();
    copy_thread_.reset();
  }

  RCLCPP_INFO(get_logger(), "Cleanup complete");
  return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
SystemDataRecorder::on_shutdown(const rclcpp_lifecycle::State & /* state */)
{
  RCLCPP_INFO(get_logger(), "Stopping and finalising recording (hard shutdown)");
  if (!cleaned_up_) {
    unsubscribe_from_topics();
    writer_.reset();
    if (copy_bags_) {
      notify_new_file_to_copy(last_bag_file_);
      notify_new_file_to_copy(source_directory_ / "metadata.yaml");
    }
  }

  if (copy_bags_ && copy_thread_) {
    notify_state_change(SdrStateChange::FINISHED);
    copy_thread_->join();
    copy_thread_.reset();
  }

  RCLCPP_INFO(get_logger(), "Cleanup complete");
  return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

void SystemDataRecorder::subscribe_to_topics()
{
  for (const auto & topic_with_type : topics_and_types_) {
    subscribe_to_topic(topic_with_type.first, topic_with_type.second);
  }
}

void SystemDataRecorder::subscribe_to_topic(const std::string &topic,
                                            const std::string &type) {
#ifdef ROS_DISTRO_HUMBLE
  // In Humble, TopicMetadata.offered_qos_profiles is a YAML string and the
  // struct has exactly four fields (name, type, serialization_format,
  // offered_qos_profiles).
  auto offered_qos = get_serialised_offered_qos_for_topic(topic);
  auto qos = get_appropriate_qos_for_topic(topic);

  auto topic_metadata = rosbag2_storage::TopicMetadata(
      {topic, type, rmw_get_serialization_format(), offered_qos});
#else
  // In Jazzy+, TopicMetadata.offered_qos_profiles is std::vector<rclcpp::QoS>
  // and the struct has additional fields (id, type_description_hash), so we use
  // named assignment.
  auto endpoints = get_publishers_info_by_topic(topic);
  auto qos = get_appropriate_qos_for_topic(topic);

  rosbag2_storage::TopicMetadata topic_metadata;
  topic_metadata.name = topic;
  topic_metadata.type = type;
  topic_metadata.serialization_format = rmw_get_serialization_format();
  for (const auto &endpoint : endpoints) {
    topic_metadata.offered_qos_profiles.push_back(endpoint.qos_profile());
  }
#endif
  writer_->create_topic(topic_metadata);

  auto subscription = create_generic_subscription(
      topic, type, qos,
      [this, topic, type](std::shared_ptr<rclcpp::SerializedMessage> message) {
        if (get_current_state().id() ==
            lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE) {
          writer_->write(message, topic, type,
                         rclcpp::Clock(RCL_SYSTEM_TIME).now());
        }
      });
  if (subscription) {
    subscriptions_.insert({topic, subscription});
    RCLCPP_INFO(get_logger(), "Subscribed to topic '%s'", topic.c_str());
  } else {
    writer_->remove_topic(topic_metadata);
    RCLCPP_ERROR(get_logger(), "Failed to subscribe to topic '%s'", topic.c_str());
  }
}

std::string SystemDataRecorder::get_serialised_offered_qos_for_topic(
    const std::string &topic) {
#ifdef ROS_DISTRO_HUMBLE
  // In Humble, TopicMetadata.offered_qos_profiles is a YAML-serialised string.
  YAML::Node offered_qos_profiles;
  auto endpoints = get_publishers_info_by_topic(topic);
  for (const auto & endpoint : endpoints) {
    offered_qos_profiles.push_back(rosbag2_transport::Rosbag2QoS(endpoint.qos_profile()));
  }
  return YAML::Dump(offered_qos_profiles);
#else
  // In Jazzy+, TopicMetadata.offered_qos_profiles is std::vector<rclcpp::QoS>.
  // This method is not used on Jazzy+; return an empty string to satisfy the
  // declaration.
  (void)topic;
  return "";
#endif
}

rclcpp::QoS SystemDataRecorder::get_appropriate_qos_for_topic(const std::string & topic)
{
  auto qos = rclcpp::QoS(rmw_qos_profile_default.depth);

  auto endpoints = get_publishers_info_by_topic(topic);
  if (endpoints.empty()) {
    return qos;
  }

  size_t reliability_reliable_endpoints_count = 0;
  size_t durability_transient_local_endpoints_count = 0;
  for (const auto & endpoint : endpoints) {
    const auto & profile = endpoint.qos_profile().get_rmw_qos_profile();
    if (profile.reliability == RMW_QOS_POLICY_RELIABILITY_RELIABLE) {
      ++reliability_reliable_endpoints_count;
    }
    if (profile.durability == RMW_QOS_POLICY_DURABILITY_TRANSIENT_LOCAL) {
      ++durability_transient_local_endpoints_count;
    }
  }

  if (reliability_reliable_endpoints_count == endpoints.size()) {
    qos.reliable();
  } else {
    if (reliability_reliable_endpoints_count > 0) {
      RCLCPP_WARN(
        get_logger(),
        "Some, but not all, publishers on topic \"%s\" are offering "
          "RMW_QOS_POLICY_RELIABILITY_RELIABLE. Falling back to "
          "RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT as it will connect to all publishers. Some "
          "messages from Reliable publishers could be dropped.",
        topic.c_str());
    }
    qos.best_effort();
  }

  if (durability_transient_local_endpoints_count == endpoints.size()) {
    qos.transient_local();
  } else {
    if (durability_transient_local_endpoints_count > 0) {
      RCLCPP_WARN(
        get_logger(),
        "Some, but not all, publishers on topic \"%s\" are offering "
          "RMW_QOS_POLICY_DURABILITY_TRANSIENT_LOCAL. Falling back to "
          "RMW_QOS_POLICY_DURABILITY_VOLATILE as it will connect to all publishers. Previously-"
          "published latched messages will not be retrieved.",
        topic.c_str());
    }
    qos.durability_volatile();
  }

  return qos;
}

void SystemDataRecorder::unsubscribe_from_topics()
{
  // Destroying the subscription objects automatically unsubscribes
  subscriptions_.clear();
}

void SystemDataRecorder::copy_thread_main()
{
  RCLCPP_INFO(get_logger(), "Copy thread: Starting");
  SdrStateChange current_state = SdrStateChange::PAUSED;
  std::queue<std::string> local_files_to_copy;
  while (current_state != SdrStateChange::FINISHED)
  {
    { // Critical section start
      std::unique_lock<std::mutex> lock(copy_thread_mutex_);
      if (files_to_copy_.empty()) {
        while (!copy_thread_should_wake()) {
          copy_thread_wake_cv_.wait(lock);
        }
      }

      if (state_msg_ != SdrStateChange::NO_CHANGE) {
        current_state = state_msg_;
        state_msg_ = SdrStateChange::NO_CHANGE;
      }

      local_files_to_copy.swap(files_to_copy_);
    } // Critical section end

    while (!local_files_to_copy.empty()) {
      std::string uri = local_files_to_copy.front();
      local_files_to_copy.pop();
      copy_bag_file(uri);
    }
  }
  RCLCPP_INFO(get_logger(), "Copy thread: Exiting");
}

bool SystemDataRecorder::copy_thread_should_wake()
{
  return state_msg_ != SdrStateChange::NO_CHANGE || !files_to_copy_.empty();
}

void SystemDataRecorder::notify_state_change(SdrStateChange new_state)
{
  {
    std::lock_guard<std::mutex> lock(copy_thread_mutex_);
    state_msg_ = new_state;
  }
  copy_thread_wake_cv_.notify_one();
}

void SystemDataRecorder::notify_new_file_to_copy(const std::string & file_uri)
{
  {
    std::lock_guard<std::mutex> lock(copy_thread_mutex_);
    files_to_copy_.push(file_uri);
  }
  copy_thread_wake_cv_.notify_one();
}

void SystemDataRecorder::notify_new_file_to_copy(const std::filesystem::path & file_path)
{
  notify_new_file_to_copy(file_path.string());
}

bool SystemDataRecorder::ensure_directory_writable(const std::filesystem::path & path)
{
  if (!std::filesystem::exists(path)) {
    try {
      std::filesystem::create_directories(path);
    } catch (const std::filesystem::filesystem_error & ex) {
      RCLCPP_ERROR(
        get_logger(),
        "Failed to create directory '%s': %s",
        path.c_str(), ex.what());
      return false;
    }
    RCLCPP_INFO(get_logger(), "Created directory: %s", path.c_str());
  }

  if (access(path.c_str(), W_OK) != 0) {
    RCLCPP_ERROR(
      get_logger(),
      "Cannot write to directory '%s': permission denied",
      path.c_str());
    return false;
  }

  return true;
}

bool SystemDataRecorder::create_copy_destination()
{
  if (std::filesystem::exists(destination_directory_)) {
    RCLCPP_ERROR(
      get_logger(),
      "Copy destination directory already exists: %s",
      destination_directory_.c_str());
    return false;
  }
  RCLCPP_INFO(
    get_logger(),
    "Creating destination directory %s",
    destination_directory_.c_str());
  return std::filesystem::create_directories(destination_directory_);
}

void SystemDataRecorder::copy_bag_file(const std::string & bag_file_name)
{
  RCLCPP_INFO(
    get_logger(),
    "Copying %s to %s",
    bag_file_name.c_str(),
    destination_directory_.c_str());

  std::filesystem::copy(bag_file_name, destination_directory_);
}

}  // namespace sdr
