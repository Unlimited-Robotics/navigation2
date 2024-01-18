// Copyright (c) 2018 Samsung Research America
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

#include <string>
#include <memory>

#include "nav2_behavior_tree/plugins/action/publish_feedback.hpp"
//home/brasteur/ur_dev_sim/ws_gary_sim/src/raya/raya_cmd_msgs/msg/CmdFeedbackHeader.msg
namespace nav2_behavior_tree
{

PublishFeedback::PublishFeedback(
  const std::string & name,
  const BT::NodeConfiguration & conf)
: BT::SyncActionNode(name, conf)
{
  node_ = config().blackboard->get<rclcpp::Node::SharedPtr>("node");
  feedback_initialized_ = config().blackboard->get<bool>("feedback_initialized");
  if(feedback_initialized_){
    feedback_publisher = config().blackboard->get<rclcpp::Publisher<raya_cmd_msgs::msg::CmdFeedbackHeader>::SharedPtr>("feedback_publisher");
  }else{
    RCLCPP_ERROR(node_->get_logger(), "Enable feedback was never used...");
    throw BT::RuntimeError("Enable feedback was never used...");
  }
}

BT::NodeStatus PublishFeedback::tick()
{
  int error_code;
  std::string error_msg;
  uint feedback_code;
  getInput("error_code", error_code);
  getInput("error_msg", error_msg);
  feedback_code = abs(error_code);

  auto feedback_state = FeedbackMsg_();
  feedback_state.feedback_code = feedback_code;
  feedback_state.feedback_msg = error_msg;
  feedback_publisher->publish(feedback_state);

  return BT::NodeStatus::SUCCESS;
}

}  // namespace nav2_behavior_tree

#include "behaviortree_cpp_v3/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<nav2_behavior_tree::PublishFeedback>("PublishFeedback");
}
