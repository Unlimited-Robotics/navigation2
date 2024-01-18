#include <string>
#include <memory>

#include "nav2_behavior_tree/plugins/action/rotate_action.hpp"

namespace nav2_behavior_tree
{

RotateAction::RotateAction(
  const std::string & xml_tag_name,
  const std::string & action_name,
  const BT::NodeConfiguration & conf)
: BtActionNode<raya_motion_msgs::action::SigRotate>(xml_tag_name, action_name, conf)
{

  node_ = config().blackboard->get<rclcpp::Node::SharedPtr>("node");
  feedback_publisher = config().blackboard->get<rclcpp::Publisher<raya_cmd_msgs::msg::CmdFeedbackHeader>::SharedPtr>("feedback_publisher");
  error_publisher = config().blackboard->get<rclcpp::Publisher<raya_cmd_msgs::msg::CmdFeedbackHeader>::SharedPtr>("error_publisher");
  RCLCPP_WARN(
      node_->get_logger(), "Setting rotate");
  
}

BT::NodeStatus RotateAction::on_aborted()
{
  RCLCPP_WARN(
      node_->get_logger(), "ROTATE ABORTED");

  auto error_state = FeedbackMsg_();
  error_state.feedback_code = 115;
  error_state.feedback_msg = "Couldn't rotate, obstacle detected...";
  error_publisher->publish(error_state);

  return BT::NodeStatus::FAILURE;
}

void RotateAction::on_tick()
{
  RCLCPP_WARN(
      node_->get_logger(), "Rotate called");

  auto feedback_state = FeedbackMsg_();
  feedback_state.feedback_code = 30;
  feedback_state.feedback_msg = "Rotate called";
  feedback_publisher->publish(feedback_state);

  float angle;
  float angular_velocity;
  bool use_obstacles;
  getInput("angle", angle);
  getInput("angular_velocity", angular_velocity);
  getInput("enable_obstacles", use_obstacles);

  RCLCPP_WARN(
    node_->get_logger(), "Angle target = "
    "(%f).", angle);

  goal_.angle_offset = angle;
  goal_.angular_speed = angular_velocity;
  goal_.enable_obstacles = use_obstacles;
  goal_.absolute_angle = false;

  feedback_state.feedback_code = 30;
  feedback_state.feedback_msg = "Rotating to angle";
  feedback_publisher->publish(feedback_state);
}


}  // namespace nav2_behavior_tree

#include "behaviortree_cpp_v3/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  BT::NodeBuilder builder =
    [](const std::string & name, const BT::NodeConfiguration & config)
    {
      return std::make_unique<nav2_behavior_tree::RotateAction>(name, "/gary/motion/signal/rotate", config);
    };

  factory.registerBuilder<nav2_behavior_tree::RotateAction>("Rotate", builder);
}
