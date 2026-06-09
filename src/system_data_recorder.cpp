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

#include <condition_variable>
#include <mutex>
#include <thread>

#include "sdr/sdr_component.hpp"

#include "rclcpp/rclcpp.hpp"
#include "lifecycle_msgs/msg/state.hpp"
#include "lifecycle_msgs/msg/transition.hpp"

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);

  std::shared_ptr<sdr::SystemDataRecorder> sdr_component;
  try {
    sdr_component = std::make_shared<sdr::SystemDataRecorder>(
      "sdr",
      rclcpp::NodeOptions());
  } catch (const std::exception & ex) {
    RCLCPP_FATAL(
      rclcpp::get_logger("system_data_recorder"),
      "Failed to create SystemDataRecorder node: %s", ex.what());
    rclcpp::shutdown();
    return 1;
  }

  // One-shot timer: trigger configure after the executor has started
  // spinning so the node is fully live on the graph before lifecycle transitions.
  rclcpp::TimerBase::SharedPtr init_timer;
  if (sdr_component->get_parameter("autostart").as_bool()) {
    init_timer = sdr_component->create_wall_timer(
      std::chrono::milliseconds(1000),
      [&sdr_component, &init_timer]() {
        init_timer->cancel();  // fire only once

        // Transition: Unconfigured -> Inactive
        auto configure_result = sdr_component->trigger_transition(
          lifecycle_msgs::msg::Transition::TRANSITION_CONFIGURE);
        if (configure_result.id() != lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE) {
          RCLCPP_ERROR(
            sdr_component->get_logger(),
            "Failed to configure SystemDataRecorder (resulting state id: %u)",
            configure_result.id());
        }
      });
  }

  rclcpp::executors::SingleThreadedExecutor exec;
  exec.add_node(sdr_component->get_node_base_interface());
  exec.spin();

  rclcpp::shutdown();
  return 0;
}
