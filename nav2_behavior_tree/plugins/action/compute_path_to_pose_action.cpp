// Copyright (c) 2018 Intel Corporation
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

#include <memory>
#include <string>

#include "nav2_behavior_tree/plugins/action/compute_path_to_pose_action.hpp"

namespace nav2_behavior_tree
{

ComputePathToPoseAction::ComputePathToPoseAction(
  const std::string & xml_tag_name,
  const std::string & action_name,
  const BT::NodeConfiguration & conf)
: BtActionNode<nav2_msgs::action::ComputePathToPose>(xml_tag_name, action_name, conf)
{
  config().blackboard->set<int>("computed_paths", 0);
  node_ = config().blackboard->get<rclcpp::Node::SharedPtr>("node");
  feedback_publisher = config().blackboard->get<rclcpp::Publisher<raya_cmd_msgs::msg::CmdFeedbackHeader>::SharedPtr>("feedback_publisher");
  error_publisher = config().blackboard->get<rclcpp::Publisher<raya_cmd_msgs::msg::CmdFeedbackHeader>::SharedPtr>("error_publisher");
}

void ComputePathToPoseAction::on_tick()
{
  getInput("goal", goal_.goal);
  getInput("planner_id", goal_.planner_id);
  if (getInput("start", goal_.start)) {
    goal_.use_start = true;
  }
}

BT::NodeStatus ComputePathToPoseAction::on_success()
{
  int computed_paths = 0;
  config().blackboard->template get<int>("computed_paths", computed_paths);
  computed_paths += 1;
  if (computed_paths >= 5){
    computed_paths = 0;
    reset_recovery_count();
    RCLCPP_WARN(
      node_->get_logger(), "RECOVERY COUNT RESTARTED...");
    int recovery_count1 = 0;
    config().blackboard->template get<int>("number_recoveries", recovery_count1);
    RCLCPP_WARN(
      node_->get_logger(), "RECOVERY COUNT = %d", recovery_count1);
  }
  config().blackboard->template set<int>("computed_paths", computed_paths); 
  setOutput("path", result_.result->path);
  auto feedback_state = FeedbackMsg_();
  feedback_state.feedback_code = 30;
  feedback_state.feedback_msg = "Navigating.";
  feedback_publisher->publish(feedback_state);
  return BT::NodeStatus::SUCCESS;
}

BT::NodeStatus ComputePathToPoseAction::on_aborted()
{
  nav_msgs::msg::Path empty_path;
  setOutput("path", empty_path);
  int computed_paths = 0;
  config().blackboard->template set<int>("computed_paths", computed_paths); 
  RCLCPP_WARN(
      node_->get_logger(), "COMPUTE PATH ABORTED");
  auto error_state = FeedbackMsg_();
  error_state.feedback_code = 116;
  error_state.feedback_msg = "Couldn't compute path to pose.";
  error_publisher->publish(error_state);
  return BT::NodeStatus::FAILURE;
}

BT::NodeStatus ComputePathToPoseAction::on_cancelled()
{
  nav_msgs::msg::Path empty_path;
  setOutput("path", empty_path);
  return BT::NodeStatus::SUCCESS;
}

void ComputePathToPoseAction::halt()
{
  nav_msgs::msg::Path empty_path;
  setOutput("path", empty_path);
  BtActionNode::halt();
}

}  // namespace nav2_behavior_tree

#include "behaviortree_cpp_v3/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  BT::NodeBuilder builder =
    [](const std::string & name, const BT::NodeConfiguration & config)
    {
      return std::make_unique<nav2_behavior_tree::ComputePathToPoseAction>(
        name, "compute_path_to_pose", config);
    };

  factory.registerBuilder<nav2_behavior_tree::ComputePathToPoseAction>(
    "ComputePathToPose", builder);
}
