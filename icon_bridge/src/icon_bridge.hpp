/*
 * Copyright (C) 2026 Intrinsic Innovation LLC
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
 */

#ifndef BRIDGES_ICON_BRIDGE_HPP_
#define BRIDGES_ICON_BRIDGE_HPP_

#include <memory>

#include "flowstate_ros_bridge/bridge_interface.hpp"
#include "icon_bridge/agent_bridge_info.h"
#include "icon_bridge/agent_bridge_joint_info.h"
#include "intrinsic/icon/cc_client/client.h"
#include "intrinsic/icon/cc_client/session.h"
#include "intrinsic/icon/cc_client/stream.h"
#include "proto/agent_bridge.pb.h"
#include "rclcpp/rclcpp.hpp"

// Interfaces
#include "aic_control_interfaces/msg/controller_state.hpp"
#include "aic_control_interfaces/msg/joint_motion_update.hpp"
#include "aic_control_interfaces/msg/motion_update.hpp"
#include "aic_control_interfaces/msg/target_mode.hpp"
#include "aic_control_interfaces/msg/trajectory_generation_mode.hpp"
#include "aic_control_interfaces/srv/change_target_mode.hpp"

namespace flowstate_ros_bridge {

using ::intrinsic::icon::Action;
using ::intrinsic::icon::ActionDescriptor;
using ::intrinsic::icon::ActionInstanceId;
using ::intrinsic::icon::Client;
using ::intrinsic::icon::OperationalStatus;
using ::intrinsic::icon::Session;

//==============================================================================
const intrinsic::icon::ActionInstanceId kAgentBridgeId(1);
const intrinsic::icon::ActionInstanceId kAgentBridgeJointId(2);

///=============================================================================
class IconBridge : public BridgeInterface {
 public:
  /// Documentation inherited.
  void declare_ros_parameters(ROSNodeInterfaces ros_node_interfaces) final;

  /// Documentation inherited.
  bool initialize(ROSNodeInterfaces ros_node_interfaces,
                  std::shared_ptr<Executive> executive_client,
                  std::shared_ptr<World> world_client) final;

 private:
  void MotionUpdateCallback(
      const aic_control_interfaces::msg::MotionUpdate::SharedPtr msg);

  void JointMotionUpdateCallback(
      const aic_control_interfaces::msg::JointMotionUpdate::SharedPtr msg);

  void ChangeTargetModeCallback(
      const std::shared_ptr<
          aic_control_interfaces::srv::ChangeTargetMode::Request>
          request,
      std::shared_ptr<aic_control_interfaces::srv::ChangeTargetMode::Response>
          response);

  //   void PollActionStateCallback();

  struct Data : public std::enable_shared_from_this<Data> {
    ROSNodeInterfaces node_interfaces_;

    std::unique_ptr<Session> session_;
    intrinsic::icon::AgentBridgeInfo::FixedParams agent_bridge_fixed_params_;
    intrinsic::icon::AgentBridgeJointInfo::FixedParams
        agent_bridge_joint_fixed_params_;
    std::optional<Action> agent_bridge_action_;
    std::optional<Action> agent_bridge_joint_action_;
    std::unique_ptr<intrinsic::icon::StreamWriterInterface<
        intrinsic_proto::icon::actions::proto::MotionUpdate>>
        agent_bridge_writer_;
    std::unique_ptr<intrinsic::icon::StreamWriterInterface<
        intrinsic_proto::icon::actions::proto::JointMotionUpdate>>
        agent_bridge_joint_writer_;

    rclcpp::Subscription<aic_control_interfaces::msg::MotionUpdate>::SharedPtr
        motion_update_sub_;
    rclcpp::Subscription<aic_control_interfaces::msg::JointMotionUpdate>::
        SharedPtr joint_motion_update_sub_;
    rclcpp::Service<aic_control_interfaces::srv::ChangeTargetMode>::SharedPtr
        change_target_mode_srv_;

    rclcpp::Publisher<aic_control_interfaces::msg::ControllerState>::SharedPtr
        controller_state_pub_;

    std::string part_name_;
    std::size_t num_joints_;
    uint8_t target_mode_value_ =
        aic_control_interfaces::msg::TargetMode::MODE_UNSPECIFIED;

    ~Data();
  };
  std::shared_ptr<Data> data_;
};
}  // namespace flowstate_ros_bridge.

#endif  // BRIDGES_ICON_BRIDGE_HPP_
