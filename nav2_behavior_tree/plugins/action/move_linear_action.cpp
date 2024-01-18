#include <string>
#include <memory>

#include "nav2_behavior_tree/plugins/action/move_linear_action.hpp"

namespace nav2_behavior_tree
{

MoveLinearAction::MoveLinearAction(
  const std::string & xml_tag_name,
  const std::string & action_name,
  const BT::NodeConfiguration & conf)
: BtActionNode<raya_motion_msgs::action::SigMoveLinear>(xml_tag_name, action_name, conf)
{

  node_ = config().blackboard->get<rclcpp::Node::SharedPtr>("node");
  feedback_publisher = config().blackboard->get<rclcpp::Publisher<raya_cmd_msgs::msg::CmdFeedbackHeader>::SharedPtr>("feedback_publisher");
  error_publisher = config().blackboard->get<rclcpp::Publisher<raya_cmd_msgs::msg::CmdFeedbackHeader>::SharedPtr>("error_publisher");
  RCLCPP_WARN(
      node_->get_logger(), "Setting move linear");
  
}

BT::NodeStatus MoveLinearAction::on_aborted()
{
  RCLCPP_WARN(
      node_->get_logger(), "MOVE LINEAR ABORTED");

  auto error_state = FeedbackMsg_();
  error_state.feedback_code = 115;
  error_state.feedback_msg = "Couldn't move linear, obstacle detected...";
  error_publisher->publish(error_state);

  return BT::NodeStatus::FAILURE;
}

void MoveLinearAction::on_tick()
{
  RCLCPP_WARN(
      node_->get_logger(), "Move linear called");

  auto feedback_state = FeedbackMsg_();
  feedback_state.feedback_code = 20;
  feedback_state.feedback_msg = "moving in a straight line.";
  feedback_publisher->publish(feedback_state);

  float distance;
  float velocity;
  bool enable_obstacles;
  getInput("distance", distance);
  getInput("velocity", velocity);
  getInput("enable_obstacles", enable_obstacles);

  goal_.x_velocity = velocity;
  goal_.distance = distance;
  goal_.enable_obstacles = enable_obstacles;

  feedback_state.feedback_code = 30;
  feedback_state.feedback_msg = "Moving linear";
  feedback_publisher->publish(feedback_state);
}

}  // namespace nav2_behavior_tree

#include "behaviortree_cpp_v3/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  BT::NodeBuilder builder =
    [](const std::string & name, const BT::NodeConfiguration & config)
    {
      return std::make_unique<nav2_behavior_tree::MoveLinearAction>(name, "/gary/motion/signal/move_linear", config);
    };

  factory.registerBuilder<nav2_behavior_tree::MoveLinearAction>("MoveLinear", builder);
}
