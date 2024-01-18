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

#ifndef NAV2_BEHAVIOR_TREE__PLUGINS__ACTION__PUBLISH_FEEDBACK_ACTION_HPP_
#define NAV2_BEHAVIOR_TREE__PLUGINS__ACTION__PUBLISH_FEEDBACK_ACTION_HPP_

#include <string>

#include "nav2_behavior_tree/bt_action_node.hpp"
#include "nav2_util/simple_action_server.hpp"
#include "nav2_msgs/action/navigate_to_pose.hpp"
#include "raya_cmd_msgs/msg/cmd_feedback_header.hpp"  


namespace nav2_behavior_tree
{

  using NavigationGeneralAction = nav2_msgs::action::NavigateToPose;
  using NavigationGeneralActionServer = nav2_util::SimpleActionServer<NavigationGeneralAction>;

/**
 * @brief A nav2_behavior_tree::BtActionNode class that wraps nav2_msgs::action::Wait
 */
class PublishFeedback : public BT::SyncActionNode
{
public:
  /**
   * @brief A constructor for nav2_behavior_tree::PublishFeedback
   * @param xml_tag_name Name for the XML tag for this node
   * @param conf BT node configuration
   */
  PublishFeedback(
    const std::string & xml_tag_name,
    const BT::NodeConfiguration & conf);

  /**
   * @brief Creates list of BT ports
   * @return BT::PortsList Containing basic ports along with node-specific ports
   */
  static BT::PortsList providedPorts()
  {
    return {
      BT::InputPort<int>("error_code", 0, "error_code"),
      BT::InputPort<std::string>("error_msg", "", "error_msgs")
    };
  }

private:

  BT::NodeStatus tick() override;
  
  rclcpp::Node::SharedPtr node_;
  rclcpp::CallbackGroup::SharedPtr callback_group_;
  rclcpp::executors::SingleThreadedExecutor callback_group_executor_;
  std::shared_ptr<NavigationGeneralActionServer> action_server_;
  using FeedbackMsg_ = raya_cmd_msgs::msg::CmdFeedbackHeader;
  rclcpp::Publisher<FeedbackMsg_>::SharedPtr feedback_publisher;
  bool feedback_initialized_;
};

}  // namespace nav2_behavior_tree

#endif  // NAV2_BEHAVIOR_TREE__PLUGINS__ACTION__PUBLISH_FEEDBACK_ACTION_HPP_
