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

#ifndef NAV2_BEHAVIOR_TREE__PLUGINS__ACTION__MOVE_LINEAR_ACTION_HPP_
#define NAV2_BEHAVIOR_TREE__PLUGINS__ACTION__MOVE_LINEAR_ACTION_HPP_

#include <string>

#include "nav2_behavior_tree/bt_action_node.hpp"
#include "raya_navigation_msgs/action/sig_go_to_angle.hpp" 
#include "raya_motion_msgs/action/sig_move_linear.hpp"  
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
class MoveLinearAction : public BtActionNode<raya_motion_msgs::action::SigMoveLinear>
{
public:
  /**
   * @brief A constructor for nav2_behavior_tree::MoveLinearAction
   * @param xml_tag_name Name for the XML tag for this node
   * @param action_name Action name this node creates a client for
   * @param conf BT node configuration
   */
  MoveLinearAction(
    const std::string & xml_tag_name,
    const std::string & action_name,
    const BT::NodeConfiguration & conf);

  /**
   * @brief Function to perform some user-defined operation on tick
   */
  void on_tick() override;

  BT::NodeStatus on_aborted() override;
  
  /**
   * @brief Creates list of BT ports
   * @return BT::PortsList Containing basic ports along with node-specific ports
   */
  static BT::PortsList providedPorts()
  {
    return providedBasicPorts(
      {
        BT::InputPort<float>("distance", 0.2, "distance in meters"),
        BT::InputPort<float>("velocity", -0.1, "positive move fordward negative move backward"),
        BT::InputPort<bool>("enable_obstacles", true, "true if consider obstacles")
      });
  }

private:

  rclcpp::Node::SharedPtr node_;
  using FeedbackMsg_ = raya_cmd_msgs::msg::CmdFeedbackHeader;
  rclcpp::Publisher<FeedbackMsg_>::SharedPtr feedback_publisher;
  rclcpp::Publisher<FeedbackMsg_>::SharedPtr error_publisher;

};

}  // namespace nav2_behavior_tree

#endif  // NAV2_BEHAVIOR_TREE__PLUGINS__ACTION__MOVE_LINEAR_ACTION_HPP_
