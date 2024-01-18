// Copyright (c) 2019 Samsung Research America
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

#include <cmath>
#include <chrono>
#include <memory>

#include "nav2_behaviors/plugins/wait.hpp"

namespace nav2_behaviors
{

Wait::Wait()
: TimedBehavior<WaitAction>(),
  feedback_(std::make_shared<WaitAction::Feedback>())
{
}

Wait::~Wait() = default;

Status Wait::onRun(const std::shared_ptr<const WaitAction::Goal> command)
{
  wait_end_ = node_.lock()->now() + rclcpp::Duration(command->time);
  auto ros_node_ = rclcpp::Node::make_shared("wait_node_");
  rclcpp::QoS qos(rclcpp::KeepLast(10));
  
  qos.reliability(RMW_QOS_POLICY_RELIABILITY_RELIABLE);
  qos.durability(RMW_QOS_POLICY_DURABILITY_VOLATILE);
  feedback_publisher =
    ros_node_->create_publisher<raya_cmd_msgs::msg::CmdFeedbackHeader>(
        "/nav2_feedback_state", qos
      );  
  return Status::SUCCEEDED;
}

Status Wait::onCycleUpdate()
{
  auto current_point = node_.lock()->now();
  auto time_left = wait_end_ - current_point;

  feedback_->time_left = time_left;
  action_server_->publish_feedback(feedback_);

  if (time_left.nanoseconds() > 0) {
    auto error_state = FeedbackMsg_();
    error_state.feedback_code = 10;
    error_state.feedback_msg = "Waiting.......";
    feedback_publisher->publish(error_state);
    return Status::RUNNING;
  } else {
    return Status::SUCCEEDED;
  }
}

}  // namespace nav2_behaviors

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(nav2_behaviors::Wait, nav2_core::Behavior)
