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

#ifndef SDR__SDR_COMPONENT_HPP__
#define SDR__SDR_COMPONENT_HPP__

#include <condition_variable>
#include <filesystem>
#include <mutex>
#include <queue>
#include <string>
#include <thread>
#include <unordered_map>

#include "sdr/visibility_control.h"

#include "rclcpp/node_options.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "rosbag2_cpp/writer.hpp"
#include "rosbag2_transport/recorder.hpp"

namespace sdr
{

class SystemDataRecorder : public rclcpp_lifecycle::LifecycleNode
{
public:
  /// Constructor.
  ///
  /// All recording configuration is supplied through ROS 2 parameters.  The preferred
  /// way is a YAML config file (default: config/sdr.yaml); individual parameters can
  /// also be set via ROS 2 CLI arguments (--ros-args -p <name>:=<value>).
  ///
  /// Required parameters
  ///   bags_dir        (string)  – root directory where bags are written
  ///
  /// Optional parameters (defaults listed)
  ///   session_name    (string, "")     – subdirectory/file prefix; defaults to yyyymmdd_HHMMSS
  ///   storage_type    (string, "mcap") – "mcap" or "sqlite3"
  ///   max_file_size   (int,    1024)   – maximum bag-file size in MB; 0 = unlimited
  ///   autostart       (bool,   false)  – trigger CONFIGURE automatically after 1 s
  ///   copy_bags       (bool,   false)  – copy closed bag files to copy_dir
  ///   copy_dir        (string, "")     – destination for copies (required when copy_bags=true)
  ///   topic_names     (string[], ["/parameter_events", "/rosout"])
  ///   topic_types     (string[], ["rcl_interfaces/msg/ParameterEvent", "rcl_interfaces/msg/Log"])
  SDR_PUBLIC
  SystemDataRecorder(
    const std::string & node_name,
    const rclcpp::NodeOptions & options);

  SDR_PUBLIC
  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
  on_configure(const rclcpp_lifecycle::State & state);

  SDR_PUBLIC
  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
  on_activate(const rclcpp_lifecycle::State & state);

  SDR_PUBLIC
  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
  on_deactivate(const rclcpp_lifecycle::State & state);

  SDR_PUBLIC
  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
  on_cleanup(const rclcpp_lifecycle::State & state);

  SDR_PUBLIC
  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
  on_shutdown(const rclcpp_lifecycle::State & state);

private:
  // Options for the bag storage
  rosbag2_storage::StorageOptions storage_options_;
  // The topics that will be recorded, and their types
  std::unordered_map<std::string, std::string> topics_and_types_;
  // A map from topic names to subscriber entities
  std::unordered_map<std::string, std::shared_ptr<rclcpp::GenericSubscription>> subscriptions_;

  // This writer does the actual storing of data during recording
  std::shared_ptr<rosbag2_cpp::Writer> writer_;

  void start_recording();
  void stop_recording();
  void pause_recording();
  void unpause_recording();

  void subscribe_to_topics();
  void subscribe_to_topic(const std::string & topic, const std::string & type);
  std::string get_serialised_offered_qos_for_topic(const std::string & topic);
  rclcpp::QoS get_appropriate_qos_for_topic(const std::string & topic);
  void unsubscribe_from_topics();

  // This is used to pass state-change messages from the node to the file-copying worker thread
  enum class SdrStateChange
  {
    NO_CHANGE,
    PAUSED,
    RECORDING,
    FINISHED
  };

  // This variable holds the message to send to the worker thread
  SdrStateChange state_msg_ = SdrStateChange::NO_CHANGE;
  // This queue passes bag files that need to be copied to the worker thread
  std::queue<std::string> files_to_copy_;
  // This mutex protects the two above variables
  std::mutex copy_thread_mutex_;
  // This condition variable is used to wake up the worker thread when there is a new state-change
  // message or there are files to copy
  std::condition_variable copy_thread_wake_cv_;
  // The worker thread
  std::shared_ptr<std::thread> copy_thread_;

  void copy_thread_main();
  bool copy_thread_should_wake();
  void notify_state_change(SdrStateChange new_state);
  void notify_new_file_to_copy(const std::string & file_uri);
  void notify_new_file_to_copy(const std::filesystem::path & file_path);

  // The source directory is the location of the bag, i.e. where bag files are copied from
  std::filesystem::path source_directory_;
  // The destination directory where bag files will be copied to
  std::filesystem::path destination_directory_;
  // The name of the final file in the bag; no event notification will be received when recording
  // stops because there is no "stop recording" concept in rosbag2, you simply stop passing data to
  // the writer. So we need to store the name of the final file and copy it during cleanup.
  std::string last_bag_file_ = "";
  // Prevent multiple cleanup
  bool cleaned_up_ = true;

  // Whether to copy closed bag files to a destination directory
  bool copy_bags_ = false;

  // Ensure a directory exists and is writable; creates it if missing.
  // Returns true on success, false (and logs an error) on failure.
  bool ensure_directory_writable(const std::filesystem::path & path);

  bool create_copy_destination();
  void copy_bag_file(const std::string & bag_file_name);
};

}  // namespace sdr

#endif  // SDR__SDR_COMPONENT_HPP__
