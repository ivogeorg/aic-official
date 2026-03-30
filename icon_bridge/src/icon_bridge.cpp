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

#include "icon_bridge.hpp"

#include <string>

#include "absl/log/log.h"
#include "intrinsic/icon/cc_client/operational_status.h"
#include "intrinsic/icon/common/builtins.h"
#include "intrinsic/util/grpc/channel.h"
#include "intrinsic/util/grpc/connection_params.h"
#include "intrinsic/util/proto/get_text_proto.h"
#include "rclcpp/create_service.hpp"
#include "rclcpp/parameter.hpp"

namespace flowstate_ros_bridge {
constexpr const char* kServerAddressParamName = "server_address";
constexpr const char* kInstanceParamName = "instance";
constexpr const char* kPartNameParamName = "part_name";
constexpr const char* kAgentBridgeTaskSettingsFileParamName =
    "agent_bridge_task_settings_file";
constexpr const char* kAgentBridgeJointTaskSettingsFileParamName =
    "agent_bridge_joint_task_settings_file";

///=============================================================================
void IconBridge::declare_ros_parameters(ROSNodeInterfaces ros_node_interfaces) {
  const auto& param_interface =
      ros_node_interfaces
          .get<rclcpp::node_interfaces::NodeParametersInterface>();

  param_interface->declare_parameter(kServerAddressParamName,
                                     rclcpp::ParameterValue{"localhost:17080"});
  param_interface->declare_parameter(
      kInstanceParamName, rclcpp::ParameterValue{"robot_controller"});
  param_interface->declare_parameter(kPartNameParamName,
                                     rclcpp::ParameterValue{"arm"});
  param_interface->declare_parameter(kAgentBridgeTaskSettingsFileParamName,
                                     rclcpp::ParameterValue{""});
  param_interface->declare_parameter(kAgentBridgeJointTaskSettingsFileParamName,
                                     rclcpp::ParameterValue{""});
}

///=============================================================================
bool IconBridge::initialize(ROSNodeInterfaces ros_node_interfaces,
                            std::shared_ptr<Executive> /*executive_client*/,
                            std::shared_ptr<World> /*world_client*/) {
  data_ = std::make_shared<Data>();

  data_->node_interfaces_ = std::move(ros_node_interfaces);

  const auto& param_interface =
      data_->node_interfaces_
          .get<rclcpp::node_interfaces::NodeParametersInterface>();

  std::string server_address =
      param_interface->get_parameter(kServerAddressParamName)
          .get_value<std::string>();
  std::string instance = param_interface->get_parameter(kInstanceParamName)
                             .get_value<std::string>();
  data_->part_name_ = param_interface->get_parameter(kPartNameParamName)
                          .get_value<std::string>();
  std::string agent_bridge_task_settings_file =
      param_interface->get_parameter(kAgentBridgeTaskSettingsFileParamName)
          .get_value<std::string>();
  std::string agent_bridge_joint_task_settings_file =
      param_interface->get_parameter(kAgentBridgeJointTaskSettingsFileParamName)
          .get_value<std::string>();

  LOG(INFO) << "Connecting to ICON server: " << server_address
            << ", instance: " << instance << ", part: " << data_->part_name_;

  auto connection_params =
      intrinsic::ConnectionParams::ResourceInstance(instance, server_address);
  auto icon_channel_or = intrinsic::Channel::MakeFromAddress(connection_params);
  if (!icon_channel_or.ok()) {
    LOG(ERROR) << "Failed to create ICON channel: "
               << icon_channel_or.status().message().data();
    throw std::runtime_error("Failed to create ICON channel");
  }

  Client icon_client(icon_channel_or.value());

  // Check operation status of ICON Server
  absl::StatusOr<OperationalStatus> operational_status =
      icon_client.GetOperationalStatus();
  if (!operational_status.ok()) {
    LOG(ERROR) << "Failed to retrieve operational status of ICON Server.";
    throw std::runtime_error(
        "Failed to retrieve operational status of ICON Server.");
  }
  switch (operational_status->state()) {
    case intrinsic::icon::OperationalState::kDisabled:
      LOG(ERROR) << "ICON Server operational status: Disabled!";
      throw std::runtime_error("ICON Server operational status is disabled.");
      break;
    case intrinsic::icon::OperationalState::kFaulted:
      LOG(ERROR) << "ICON Server operational status: Faulted! Reason: "
                 << operational_status->fault_reason();
      throw std::runtime_error("ICON Server operational status is faulted.");
      break;
    case intrinsic::icon::OperationalState::kEnabled:
      LOG(INFO) << "ICON Server operational status: Enabled.";
      break;
    default:
      LOG(INFO) << "ICON Server operational status: Unknown.";
      throw std::runtime_error("ICON Server operational status is unknown.");
  }

  // Get pose of TCP
  absl::StatusOr<intrinsic_proto::icon::PartStatus> part_status =
      icon_client.GetSinglePartStatus(data_->part_name_);
  if (!part_status.ok()) {
    LOG(ERROR) << "Failed to retrieve part status from ICON Server.";
    throw std::runtime_error(
        "Failed to retrieve part status from ICON Server.");
  }
  data_->num_joints_ = part_status.value().joint_states_size();
  auto base_t_tip_sensed = part_status.value().base_t_tip_sensed();

  // Start ICON Session
  auto session_or =
      Session::Start(icon_channel_or.value(), {data_->part_name_});
  if (!session_or.ok()) {
    LOG(ERROR) << "Failed to start ICON session: "
               << session_or.status().message().data();
    throw std::runtime_error("Failed to start ICON session");
  }
  data_->session_ = std::move(session_or.value());

  // Load TaskSettings from file
  if (!agent_bridge_task_settings_file.empty()) {
    auto status = intrinsic::GetTextProto(
        agent_bridge_task_settings_file,
        *(data_->agent_bridge_fixed_params_.mutable_task_settings()));
    if (!status.ok()) {
      LOG(ERROR) << "Failed to start load task settings for AgentBridge: "
                 << status.message().data();
      throw std::runtime_error("Failed to load task settings for AgentBridge");
    }
  }
  if (!agent_bridge_joint_task_settings_file.empty()) {
    auto status = intrinsic::GetTextProto(
        agent_bridge_joint_task_settings_file,
        *(data_->agent_bridge_joint_fixed_params_.mutable_task_settings()));
    if (!status.ok()) {
      LOG(ERROR) << "Failed to start load task settings for AgentBridgeJoint: "
                 << status.message().data();
      throw std::runtime_error(
          "Failed to load task settings for AgentBridgeJoint");
    }
  }

  // Initialize Motion Update defaults
  auto* initial_motion_update =
      data_->agent_bridge_fixed_params_.mutable_motion_update();
  if (!initial_motion_update->has_target_state()) {
    initial_motion_update->set_trajectory_generation_mode(
        intrinsic_proto::icon::actions::proto::TrajectoryGenerationMode::
            VELOCITY);
    auto* target_state = initial_motion_update->mutable_target_state();
    target_state->mutable_velocity()->Resize(6, 0.0);

    // Use max values from task settings for initial stiffness, damping, and
    // mass
    auto* target_stiffness = initial_motion_update->mutable_target_stiffness();
    auto* target_damping = initial_motion_update->mutable_target_damping();
    auto* target_mass = initial_motion_update->mutable_target_mass();

    target_stiffness->mutable_data()->Resize(36, 0.0);
    target_damping->mutable_data()->Resize(36, 0.0);
    target_mass->mutable_data()->Resize(36, 0.0);

    if (data_->agent_bridge_fixed_params_.task_settings().has_max_stiffness() &&
        data_->agent_bridge_fixed_params_.task_settings().has_max_damping() &&
        data_->agent_bridge_fixed_params_.task_settings()
            .has_mass_for_critical_damping()) {
      const auto& max_stiffness =
          data_->agent_bridge_fixed_params_.task_settings().max_stiffness();
      const auto& max_damping =
          data_->agent_bridge_fixed_params_.task_settings().max_damping();
      const auto& critical_mass =
          data_->agent_bridge_fixed_params_.task_settings()
              .mass_for_critical_damping();

      target_stiffness->set_data(0, max_stiffness.x());
      target_stiffness->set_data(7, max_stiffness.y());
      target_stiffness->set_data(14, max_stiffness.z());
      target_stiffness->set_data(21, max_stiffness.rx());
      target_stiffness->set_data(28, max_stiffness.ry());
      target_stiffness->set_data(35, max_stiffness.rz());

      target_damping->set_data(0, max_damping.x());
      target_damping->set_data(7, max_damping.y());
      target_damping->set_data(14, max_damping.z());
      target_damping->set_data(21, max_damping.rx());
      target_damping->set_data(28, max_damping.ry());
      target_damping->set_data(35, max_damping.rz());

      target_mass->set_data(0, critical_mass.x());
      target_mass->set_data(7, critical_mass.y());
      target_mass->set_data(14, critical_mass.z());
      target_mass->set_data(21, critical_mass.rx());
      target_mass->set_data(28, critical_mass.ry());
      target_mass->set_data(35, critical_mass.rz());
    } else {
      LOG(ERROR) << "Task settings missing max_stiffness, max_damping, or "
                    "mass_for_critical_damping parameters.";
      throw std::runtime_error(
          "Task settings missing max_stiffness, max_damping, or "
          "mass_for_critical_damping parameters.");
    }

    auto* fw = initial_motion_update->mutable_feedforward_wrench_at_tip();
    fw->set_x(0);
    fw->set_y(0);
    fw->set_z(0);
    fw->set_rx(0);
    fw->set_ry(0);
    fw->set_rz(0);
  }

  // Initialize Joint Motion Update defaults
  auto* initial_joint_motion_update =
      data_->agent_bridge_joint_fixed_params_.mutable_motion_update();
  initial_joint_motion_update->set_trajectory_generation_mode(
      intrinsic_proto::icon::actions::proto::TrajectoryGenerationMode::
          VELOCITY);
  initial_joint_motion_update->mutable_target_state()
      ->mutable_velocity()
      ->Resize(data_->num_joints_, 0.0);

  auto* target_joint_stiffness =
      initial_joint_motion_update->mutable_target_stiffness();
  auto* target_joint_damping =
      initial_joint_motion_update->mutable_target_damping();

  target_joint_stiffness->mutable_joints()->Resize(data_->num_joints_, 0.0);
  target_joint_damping->mutable_joints()->Resize(data_->num_joints_, 0.0);

  if (data_->agent_bridge_joint_fixed_params_.task_settings().has_stiffness() &&
      data_->agent_bridge_joint_fixed_params_.task_settings().has_damping()) {
    const auto& stiffness =
        data_->agent_bridge_joint_fixed_params_.task_settings().stiffness();
    const auto& damping =
        data_->agent_bridge_joint_fixed_params_.task_settings().damping();

    for (std::size_t i = 0; i < data_->num_joints_; i++) {
      target_joint_stiffness->set_joints(i, stiffness.joints(i));
      target_joint_damping->set_joints(i, damping.joints(i));
    }
  }

  // For both AgentBridge and AgentBridgeJoint actions, define their
  // ActionDescriptors, then add actions to the session, then create
  // StreamWriters for each of the actions.

  // Define ActionDescriptors
  ActionDescriptor agent_bridge_descriptor =
      ActionDescriptor(
          intrinsic::icon::AgentBridgeInfo::kActionTypeName, kAgentBridgeId,
          {{intrinsic::icon::AgentBridgeInfo::kSlotName, data_->part_name_}})
          .WithFixedParams(data_->agent_bridge_fixed_params_);

  ActionDescriptor agent_bridge_joint_descriptor =
      ActionDescriptor(intrinsic::icon::AgentBridgeJointInfo::kActionTypeName,
                       kAgentBridgeJointId,
                       {{intrinsic::icon::AgentBridgeJointInfo::kSlotName,
                         data_->part_name_}})
          .WithFixedParams(data_->agent_bridge_joint_fixed_params_);

  // Add the AgentBridge action to the current session
  auto agent_bridge_action_or =
      data_->session_->AddAction(agent_bridge_descriptor);
  if (!agent_bridge_action_or.ok()) {
    LOG(ERROR) << "Failed to add AgentBridge Action: "
               << agent_bridge_action_or.status().message().data();
    throw std::runtime_error("Failed to add AgentBridge Action");
  }
  data_->agent_bridge_action_ = agent_bridge_action_or.value();

  // Add the AgentBridgeJoint action to the current session
  auto agent_bridge_joint_action_or =
      data_->session_->AddAction(agent_bridge_joint_descriptor);
  if (!agent_bridge_joint_action_or.ok()) {
    LOG(ERROR) << "Failed to add AgentBridgeJoint Action: "
               << agent_bridge_joint_action_or.status().message().data();
    throw std::runtime_error("Failed to add AgentBridgeJoint Action");
  }
  data_->agent_bridge_joint_action_ = agent_bridge_joint_action_or.value();

  // Create StreamWriter for the AgentBridge action
  auto agent_bridge_writer_or =
      data_->session_
          ->StreamWriter<intrinsic_proto::icon::actions::proto::MotionUpdate>(
              data_->agent_bridge_action_.value(),
              intrinsic::icon::AgentBridgeInfo::kStreamingCommandName);
  if (!agent_bridge_writer_or.ok()) {
    LOG(ERROR)
        << "Failed to create MotionUpdate stream writer to AgentBridge action: "
        << agent_bridge_writer_or.status().message().data();
    throw std::runtime_error("Failed to create MotionUpdate stream writer");
  }
  data_->agent_bridge_writer_ = std::move(agent_bridge_writer_or.value());

  // Create StreamWriter for the AgentBridgeJoint action
  auto agent_bridge_joint_writer_or = data_->session_->StreamWriter<
      intrinsic_proto::icon::actions::proto::JointMotionUpdate>(
      data_->agent_bridge_joint_action_.value(),
      intrinsic::icon::AgentBridgeJointInfo::kStreamingCommandName);
  if (!agent_bridge_joint_writer_or.ok()) {
    LOG(ERROR) << "Failed to create JointMotionUpdate stream writer to "
                  "AgentBridgeJoint action: "
               << agent_bridge_joint_writer_or.status().message().data();
    throw std::runtime_error(
        "Failed to create JointMotionUpdate stream writer");
  }
  data_->agent_bridge_joint_writer_ =
      std::move(agent_bridge_joint_writer_or.value());

  // The default action to start on initialization is the AgentBridge action
  auto status =
      data_->session_->StartAction(data_->agent_bridge_action_.value());
  if (!status.ok()) {
    LOG(ERROR) << "Failed to start AgentBridge Action: "
               << status.message().data();
    throw std::runtime_error("Failed to start AgentBridge Action");
  }
  LOG(INFO) << "Successfully started AgentBridge Action on ICON Server";

  data_->target_mode_value_ =
      aic_control_interfaces::msg::TargetMode::MODE_CARTESIAN;

  std::shared_ptr<rclcpp::node_interfaces::NodeTopicsInterface>
      topics_interface =
          data_->node_interfaces_
              .get<rclcpp::node_interfaces::NodeTopicsInterface>();

  // Reliable QoS subscriptions for motion commands.
  rclcpp::QoS reliable_qos = rclcpp::QoS(rclcpp::KeepLast(10)).reliable();

  // ROS Subscriptions to MotionUpdate and JointMotionUpdate commands
  data_->motion_update_sub_ =
      rclcpp::create_subscription<aic_control_interfaces::msg::MotionUpdate>(
          topics_interface, "aic_controller/pose_commands", reliable_qos,
          [this](
              const aic_control_interfaces::msg::MotionUpdate::SharedPtr msg) {
            this->MotionUpdateCallback(msg);
          });

  data_->joint_motion_update_sub_ = rclcpp::create_subscription<
      aic_control_interfaces::msg::JointMotionUpdate>(
      topics_interface, "aic_controller/joint_commands", reliable_qos,
      [this](
          const aic_control_interfaces::msg::JointMotionUpdate::SharedPtr msg) {
        this->JointMotionUpdateCallback(msg);
      });

  data_->change_target_mode_srv_ =
      rclcpp::create_service<aic_control_interfaces::srv::ChangeTargetMode>(
          data_->node_interfaces_
              .get<rclcpp::node_interfaces::NodeBaseInterface>(),
          data_->node_interfaces_
              .get<rclcpp::node_interfaces::NodeServicesInterface>(),
          "aic_controller/change_target_mode",
          [this](const std::shared_ptr<
                     aic_control_interfaces::srv::ChangeTargetMode::Request>
                     request,
                 std::shared_ptr<
                     aic_control_interfaces::srv::ChangeTargetMode::Response>
                     response) {
            this->ChangeTargetModeCallback(request, response);
          },
          rclcpp::ServicesQoS(), nullptr);

  LOG(INFO) << "Initialized ICONBridge";

  return true;
}

///=============================================================================
void IconBridge::ChangeTargetModeCallback(
    const std::shared_ptr<
        aic_control_interfaces::srv::ChangeTargetMode::Request>
        request,
    std::shared_ptr<aic_control_interfaces::srv::ChangeTargetMode::Response>
        response) {
  auto target_mode_to_string = [](uint8_t mode) {
    switch (mode) {
      case aic_control_interfaces::msg::TargetMode::MODE_UNSPECIFIED:
        return "MODE_UNSPECIFIED";
      case aic_control_interfaces::msg::TargetMode::MODE_CARTESIAN:
        return "MODE_CARTESIAN";
      case aic_control_interfaces::msg::TargetMode::MODE_JOINT:
        return "MODE_JOINT";
      default:
        return "UNKNOWN_MODE";
    }
  };

  LOG(INFO) << "Received request to change target mode to: "
            << target_mode_to_string(request->target_mode.mode);

  if (request->target_mode.mode == data_->target_mode_value_) {
    LOG(INFO) << "Already in target mode: "
              << target_mode_to_string(request->target_mode.mode);
    response->success = true;
    return;
  }

  auto status = data_->session_->StopAllActions();
  if (!status.ok()) {
    LOG(ERROR) << "Failed to stop all actions on ICON Server.";
    response->success = false;
    return;
  }
  LOG(INFO) << "Stopped all actions on ICON Server.";

  if (request->target_mode.mode ==
      aic_control_interfaces::msg::TargetMode::MODE_CARTESIAN) {
    auto status =
        data_->session_->StartAction(data_->agent_bridge_action_.value());
    if (!status.ok()) {
      LOG(ERROR) << "Failed to start AgentBridge Action: "
                 << status.message().data();
      response->success = false;
      return;
    }
    LOG(INFO) << "Successfully started AgentBridge Action on ICON Server";

    data_->target_mode_value_ =
        aic_control_interfaces::msg::TargetMode::MODE_CARTESIAN;

  } else if (request->target_mode.mode ==
             aic_control_interfaces::msg::TargetMode::MODE_JOINT) {
    auto status =
        data_->session_->StartAction(data_->agent_bridge_joint_action_.value());
    if (!status.ok()) {
      LOG(ERROR) << "Failed to start AgentBridgeJoint Action: "
                 << status.message().data();
      response->success = false;
      return;
    }
    LOG(INFO) << "Successfully started AgentBridgeJoint Action on ICON Server";

    data_->target_mode_value_ =
        aic_control_interfaces::msg::TargetMode::MODE_JOINT;
  } else {
    LOG(ERROR) << "Unsupported target mode requested.";
    response->success = false;
    return;
  }

  response->success = true;
}

///=============================================================================
void IconBridge::MotionUpdateCallback(
    const aic_control_interfaces::msg::MotionUpdate::SharedPtr msg) {
  if (!data_->agent_bridge_writer_) {
    LOG(ERROR) << "MotionUpdate stream writer not initialized.";
    return;
  }

  if (data_->target_mode_value_ !=
      aic_control_interfaces::msg::TargetMode::MODE_CARTESIAN) {
    LOG(ERROR) << "Controller is not in Cartesian target mode and will not "
                  "process MotionUpdate commands.";
    return;
  }

  // Start with the default motion update
  intrinsic_proto::icon::actions::proto::MotionUpdate proto_msg =
      data_->agent_bridge_fixed_params_.motion_update();

  // Map the trajectory generation mode to supported modes POSITION and VELOCITY
  if (msg->trajectory_generation_mode.mode ==
      aic_control_interfaces::msg::TrajectoryGenerationMode::MODE_POSITION) {
    proto_msg.set_trajectory_generation_mode(
        intrinsic_proto::icon::actions::proto::TrajectoryGenerationMode::
            POSITION);
  } else if (msg->trajectory_generation_mode.mode ==
             aic_control_interfaces::msg::TrajectoryGenerationMode::
                 MODE_VELOCITY) {
    proto_msg.set_trajectory_generation_mode(
        intrinsic_proto::icon::actions::proto::TrajectoryGenerationMode::
            VELOCITY);
  } else {
    LOG(WARNING)
        << "trajectory_generation_mode is unspecified in MotionUpdate command!";
    proto_msg.set_trajectory_generation_mode(
        intrinsic_proto::icon::actions::proto::TrajectoryGenerationMode::
            TRAJECTORY_GENERATION_MODE_UNSPECIFIED);
  }

  // Map target pose and velocities
  auto* target_state = proto_msg.mutable_target_state();
  target_state->clear_pose();
  target_state->add_pose(msg->pose.position.x);
  target_state->add_pose(msg->pose.position.y);
  target_state->add_pose(msg->pose.position.z);
  target_state->add_pose(msg->pose.orientation.x);
  target_state->add_pose(msg->pose.orientation.y);
  target_state->add_pose(msg->pose.orientation.z);
  target_state->add_pose(msg->pose.orientation.w);

  target_state->clear_velocity();
  target_state->add_velocity(msg->velocity.linear.x);
  target_state->add_velocity(msg->velocity.linear.y);
  target_state->add_velocity(msg->velocity.linear.z);
  target_state->add_velocity(msg->velocity.angular.x);
  target_state->add_velocity(msg->velocity.angular.y);
  target_state->add_velocity(msg->velocity.angular.z);

  // Map stiffness and damping parameters
  auto* stiffness = proto_msg.mutable_target_stiffness();
  auto* damping = proto_msg.mutable_target_damping();
  stiffness->clear_data();
  damping->clear_data();
  for (int i = 0; i < 36; ++i) {
    stiffness->add_data(msg->target_stiffness[i]);
    damping->add_data(msg->target_damping[i]);
  }

  // Map feedforward wrench at tip
  auto* fw = proto_msg.mutable_feedforward_wrench_at_tip();
  fw->set_x(msg->feedforward_wrench_at_tip.force.x);
  fw->set_y(msg->feedforward_wrench_at_tip.force.y);
  fw->set_z(msg->feedforward_wrench_at_tip.force.z);
  fw->set_rx(msg->feedforward_wrench_at_tip.torque.x);
  fw->set_ry(msg->feedforward_wrench_at_tip.torque.y);
  fw->set_rz(msg->feedforward_wrench_at_tip.torque.z);

  // Map the wrench feedback gains at tip
  auto* gains = proto_msg.mutable_wrench_feedback_gains_at_tip();
  gains->set_x(msg->wrench_feedback_gains_at_tip[0]);
  gains->set_y(msg->wrench_feedback_gains_at_tip[1]);
  gains->set_z(msg->wrench_feedback_gains_at_tip[2]);
  gains->set_rx(msg->wrench_feedback_gains_at_tip[3]);
  gains->set_ry(msg->wrench_feedback_gains_at_tip[4]);
  gains->set_rz(msg->wrench_feedback_gains_at_tip[5]);

  // Set time_to_target_seconds to a default of 0.0, so it will reach the target
  // as fast as possible
  proto_msg.set_time_to_target_seconds(0.0);

  // Write the MotionUpdate proto message to the AgentBridge action
  auto status = data_->agent_bridge_writer_->Write(proto_msg);
  if (!status.ok()) {
    LOG(ERROR) << "Failed to write MotionUpdate to AgentBridge: "
               << status.message().data();
  }
}

///=============================================================================
void IconBridge::JointMotionUpdateCallback(
    const aic_control_interfaces::msg::JointMotionUpdate::SharedPtr msg) {
  if (!data_->agent_bridge_joint_writer_) {
    LOG(ERROR) << "JointMotionUpdate stream writer not initialized.";
    return;
  }

  if (data_->target_mode_value_ !=
      aic_control_interfaces::msg::TargetMode::MODE_JOINT) {
    LOG(ERROR) << "Controller is not in joint target mode and will not "
                  "process JointMotionUpdate commands.";
    return;
  }

  intrinsic_proto::icon::actions::proto::JointMotionUpdate proto_msg =
      data_->agent_bridge_joint_fixed_params_.motion_update();

  // Map the trajectory generation mode to supported modes POSITION and VELOCITY
  if (msg->trajectory_generation_mode.mode ==
      aic_control_interfaces::msg::TrajectoryGenerationMode::MODE_POSITION) {
    proto_msg.set_trajectory_generation_mode(
        intrinsic_proto::icon::actions::proto::TrajectoryGenerationMode::
            POSITION);
  } else if (msg->trajectory_generation_mode.mode ==
             aic_control_interfaces::msg::TrajectoryGenerationMode::
                 MODE_VELOCITY) {
    proto_msg.set_trajectory_generation_mode(
        intrinsic_proto::icon::actions::proto::TrajectoryGenerationMode::
            VELOCITY);
  } else {
    LOG(WARNING) << "trajectory_generation_mode is unspecified in "
                    "JointMotionUpdate command!";
    proto_msg.set_trajectory_generation_mode(
        intrinsic_proto::icon::actions::proto::TrajectoryGenerationMode::
            TRAJECTORY_GENERATION_MODE_UNSPECIFIED);
  }

  // Map the target positions and velocities
  auto* target_state = proto_msg.mutable_target_state();
  target_state->clear_position();
  target_state->clear_velocity();
  for (double pos : msg->target_state.positions) {
    target_state->add_position(pos);
  }
  for (double vel : msg->target_state.velocities) {
    target_state->add_velocity(vel);
  }

  // Map the stiffness and damping parameters
  auto* stiffness = proto_msg.mutable_target_stiffness();
  auto* damping = proto_msg.mutable_target_damping();
  stiffness->clear_joints();
  damping->clear_joints();
  for (double s : msg->target_stiffness) {
    stiffness->add_joints(s);
  }
  for (double d : msg->target_damping) {
    damping->add_joints(d);
  }

  // Map the feedforward torque
  auto* ff_torque = proto_msg.mutable_target_feedforward_torque();
  ff_torque->clear_joints();
  for (double t : msg->target_feedforward_torque) {
    ff_torque->add_joints(t);
  }

  // Set time_to_target_seconds to a default of 0.0, so it will reach the target
  // as fast as possible
  proto_msg.set_time_to_target_seconds(0.0);

  // Write the JointMotionUpdate proto message to the AgentBridgeJoint action
  auto status = data_->agent_bridge_joint_writer_->Write(proto_msg);
  if (!status.ok()) {
    LOG(ERROR) << "Failed to write JointMotionUpdate to AgentBridgeJoint: "
               << status.message().data();
  }
}

///=============================================================================
IconBridge::Data::~Data() {
  auto status = session_->StopAllActions();
  if (!status.ok()) {
    LOG(ERROR) << "Failed to stop all actions on ICON Server.";
  }

  session_.reset();
  agent_bridge_action_ = std::nullopt;
  agent_bridge_joint_action_ = std::nullopt;
  agent_bridge_writer_.reset();
  agent_bridge_joint_writer_.reset();

  motion_update_sub_.reset();
  joint_motion_update_sub_.reset();
  change_target_mode_srv_.reset();
}
}  // namespace flowstate_ros_bridge

#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(flowstate_ros_bridge::IconBridge,
                       flowstate_ros_bridge::BridgeInterface)
