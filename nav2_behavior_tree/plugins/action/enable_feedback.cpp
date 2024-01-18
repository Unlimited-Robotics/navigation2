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

#include "nav2_behavior_tree/plugins/action/enable_feedback.hpp"

namespace nav2_behavior_tree
{

EnableFeedback::EnableFeedback(
  const std::string & name,
  const BT::NodeConfiguration & conf)
: BT::SyncActionNode(name, conf)
{
  node_ = config().blackboard->get<rclcpp::Node::SharedPtr>("node");
  rclcpp::QoS qos(rclcpp::KeepLast(10));
  
  qos.reliability(RMW_QOS_POLICY_RELIABILITY_RELIABLE);
  qos.durability(RMW_QOS_POLICY_DURABILITY_VOLATILE);
  
  try{
    feedback_initialized_ = config().blackboard->get<bool>("feedback_initialized");
  } catch (const std::exception& e){
    config().blackboard->set<bool>("feedback_initialized", false);
    feedback_initialized_ = false;
    std::cerr << "feedback initialized not found" << e.what() << std::endl;
  }
  if(feedback_initialized_){
    feedback_publisher = config().blackboard->get<rclcpp::Publisher<raya_cmd_msgs::msg::CmdFeedbackHeader>::SharedPtr>("feedback_publisher");
    error_publisher = config().blackboard->get<rclcpp::Publisher<raya_cmd_msgs::msg::CmdFeedbackHeader>::SharedPtr>("error_publisher");
  }else{
    feedback_publisher =
        node_->create_publisher<raya_cmd_msgs::msg::CmdFeedbackHeader>(
            "/nav2_feedback_state", qos
          );
    error_publisher =
        node_->create_publisher<raya_cmd_msgs::msg::CmdFeedbackHeader>(
            "/nav2_feedback_error", qos
          );
    config().blackboard->set<rclcpp::Publisher<raya_cmd_msgs::msg::CmdFeedbackHeader>::SharedPtr>("feedback_publisher", feedback_publisher);
    config().blackboard->set<rclcpp::Publisher<raya_cmd_msgs::msg::CmdFeedbackHeader>::SharedPtr>("error_publisher", error_publisher);
    config().blackboard->set<bool>("feedback_initialized", true);
  }
  
  RCLCPP_WARN(
      node_->get_logger(), "Feedback publisher enabled");
  
}

BT::NodeStatus EnableFeedback::tick()
{
  auto feedback_state = FeedbackMsg_();
  feedback_state.feedback_code = 0;
  feedback_state.feedback_msg = "Feedback enabled";
  feedback_publisher->publish(feedback_state);

  return BT::NodeStatus::SUCCESS;
}

}  // namespace nav2_behavior_tree

#include "behaviortree_cpp_v3/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<nav2_behavior_tree::EnableFeedback>("EnableFeedback");
}
