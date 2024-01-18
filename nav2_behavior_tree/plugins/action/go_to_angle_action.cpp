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

#include "nav2_behavior_tree/plugins/action/go_to_angle_action.hpp"

namespace nav2_behavior_tree
{

GoToAngleAction::GoToAngleAction(
  const std::string & xml_tag_name,
  const std::string & action_name,
  const BT::NodeConfiguration & conf)
: BtActionNode<raya_motion_msgs::action::SigRotate>(xml_tag_name, action_name, conf)
{

  node_ = config().blackboard->get<rclcpp::Node::SharedPtr>("node");
  feedback_publisher = config().blackboard->get<rclcpp::Publisher<raya_cmd_msgs::msg::CmdFeedbackHeader>::SharedPtr>("feedback_publisher");
  error_publisher = config().blackboard->get<rclcpp::Publisher<raya_cmd_msgs::msg::CmdFeedbackHeader>::SharedPtr>("error_publisher");
  RCLCPP_WARN(
      node_->get_logger(), "Setting go to angle");
  
}

BT::NodeStatus GoToAngleAction::on_aborted()
{
  RCLCPP_WARN(
      node_->get_logger(), "GO TO ANGLE ABORTED");

  auto error_state = FeedbackMsg_();
  error_state.feedback_code = 115;
  error_state.feedback_msg = "Couldn't rotate to goal, obstacle detected...";
  error_publisher->publish(error_state);

  return BT::NodeStatus::FAILURE;
}

void GoToAngleAction::on_tick()
{
  RCLCPP_WARN(
      node_->get_logger(), "Go to angle called");

  auto feedback_state = FeedbackMsg_();

  geometry_msgs::msg::PoseStamped goal_coordinate;
  float angular_velocity;
  getInput("goal", goal_coordinate);
  getInput("angular_velocity", angular_velocity);
  auto euler = quaternionToEuler(goal_coordinate.pose.orientation.x,
                                  goal_coordinate.pose.orientation.y,
                                  goal_coordinate.pose.orientation.z,
                                  goal_coordinate.pose.orientation.w
                                  );
  RCLCPP_WARN(
    node_->get_logger(), "Angle target = "
    "(%f).", euler[2]);

  goal_.angle_offset = euler[2];
  goal_.angular_speed = angular_velocity;
  goal_.enable_obstacles = false;
  goal_.absolute_angle = true;

  feedback_state.feedback_code = 18;
  feedback_state.feedback_msg = "Rotating.";
  feedback_publisher->publish(feedback_state);
}

std::vector<float> GoToAngleAction::quaternionToEuler(float x, float y, float z, float w) 
{ 
    float ysqr = y * y; 
  
    // roll (x-axis rotation) 
    float t0 = +2.0f * (w * x + y * z); 
    float t1 = +1.0f - 2.0f * (x * x + ysqr); 
    float roll = atan2(t0, t1); 
  
    // pitch (y-axis rotation) 
    float t2 = +2.0f * (w * y - z * x); 
    t2 = t2 > 1.0f ? 1.0f : t2; 
    t2 = t2 < -1.0f ? -1.0f : t2; 
    float pitch = asin(t2); 
  
    // yaw (z-axis rotation) 
    float t3 = +2.0f * (w * z + x * y); 
    float t4 = +1.0f - 2.0f * (ysqr + z * z); 
    float yaw = atan2(t3, t4);

    std::vector<float> euler {roll, pitch, yaw};

	return euler;									   
}

}  // namespace nav2_behavior_tree

#include "behaviortree_cpp_v3/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  BT::NodeBuilder builder =
    [](const std::string & name, const BT::NodeConfiguration & config)
    {
      return std::make_unique<nav2_behavior_tree::GoToAngleAction>(name, "/gary/motion/signal/rotate", config);
    };

  factory.registerBuilder<nav2_behavior_tree::GoToAngleAction>("GoToAngle", builder);
}
