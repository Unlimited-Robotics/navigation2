// Copyright (c) 2019 Intel Corporation
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
#include <iostream>
#include "nav2_behavior_tree/plugins/control/round_robin_node.hpp"

namespace nav2_behavior_tree
{

//RoundRobinNode::RoundRobinNode(const std::string & name)
//: BT::ControlNode::ControlNode(name, {})
//{
//}

RoundRobinNode::RoundRobinNode(
  const std::string & name,
  const BT::NodeConfiguration & config)
: BT::ControlNode(name, config)
{
  std::cout << "Round robin enabled" << std::endl;
}

BT::NodeStatus RoundRobinNode::tick()
{
  const auto num_children = children_nodes_.size();

  setStatus(BT::NodeStatus::RUNNING);

  while (num_failed_children_ < num_children) {
    current_child_ = config().blackboard->template get<int>("next_recovery_child");
    current_child_idx_ = current_child_;
    TreeNode * child_node = children_nodes_[current_child_idx_];
    const BT::NodeStatus child_status = child_node->executeTick();

    switch (child_status) {
      case BT::NodeStatus::SUCCESS:
        {
          // Wrap around to the first child
          if (++current_child_idx_ >= num_children) {
            current_child_idx_ = 0;
          }
          num_failed_children_ = 0;
          ControlNode::haltChildren();
          config().blackboard->template set<int>("next_recovery_child", current_child_idx_); 
          //std::cout << "current_child_ SCC = " << current_child_ << std::endl;
          return BT::NodeStatus::SUCCESS;
        }

      case BT::NodeStatus::FAILURE:
        {
          if (++current_child_idx_ >= num_children) {
            current_child_idx_ = 0;
          }
          num_failed_children_++;
          config().blackboard->template set<int>("next_recovery_child", current_child_idx_);
          //std::cout << "current_child_ FLR = " << current_child_ << std::endl;
          break;
        }

      case BT::NodeStatus::RUNNING:
        {
          return BT::NodeStatus::RUNNING;
        }

      default:
        {
          throw BT::LogicError("Invalid status return from BT node");
        }
    }
  }

  halt();
  return BT::NodeStatus::FAILURE;
}

void RoundRobinNode::halt()
{
  ControlNode::halt();
  current_child_idx_ = 0;
  num_failed_children_ = 0;
  config().blackboard->template set<int>("next_recovery_child", current_child_idx_);
}

}  // namespace nav2_behavior_tree

BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<nav2_behavior_tree::RoundRobinNode>("RoundRobin");
}
