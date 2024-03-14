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

#include <chrono>
#include <string>

#include "nav2_behavior_tree/plugins/decorator/retry_on_failure.hpp"

namespace nav2_behavior_tree
{

RetryOnFailure::RetryOnFailure(
  const std::string & name,
  const BT::NodeConfiguration & conf)
: BT::DecoratorNode(name, conf),
  failures_(0)
{
  getInput("retries", retries_);
}

BT::NodeStatus RetryOnFailure::tick()
{
  if (status() == BT::NodeStatus::IDLE) {
    retry_ = true;
    failures_ = 0;
  }

  setStatus(BT::NodeStatus::RUNNING);

  if (retry_ || (child_node_->status() == BT::NodeStatus::RUNNING))
  {
    retry_ = false;
    const BT::NodeStatus child_state = child_node_->executeTick();

    switch (child_state) {
      case BT::NodeStatus::RUNNING:
        return BT::NodeStatus::RUNNING;

      case BT::NodeStatus::SUCCESS:
        failures_ = 0;
        return BT::NodeStatus::SUCCESS;

      case BT::NodeStatus::FAILURE:
      default:
        failures_++;
        if(failures_ > retries_)
        {
          return BT::NodeStatus::FAILURE;
        }
        else
        {
          std::cout << "CHILD FAILED, RETRYING..." << std::endl;
          std::cout << "FAILURE NUMBER = " << failures_ << std::endl;
          retry_ = true;
        }
    }
  }

  return status();
}

}  // namespace nav2_behavior_tree

#include "behaviortree_cpp_v3/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<nav2_behavior_tree::RetryOnFailure>("RetryOnFailure");
}
