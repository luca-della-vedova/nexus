/*
 * Copyright (C) 2022 Johnson & Johnson
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

#include "workcell_orchestrator.hpp"

#include "exceptions.hpp"
#include "get_joint_constraints.hpp"
#include "get_result.hpp"
#include "make_transform.hpp"
#include "serialization.hpp"
#include "set_result.hpp"
#include "signals.hpp"
#include "transform_pose.hpp"

#include <nexus_capabilities/context.hpp>
#include <nexus_capabilities/exceptions.hpp>
#include <nexus_capabilities/utils.hpp>
#include <nexus_common/logging.hpp>
#include <nexus_common/pausable_sequence.hpp>
#include <nexus_orchestrator_msgs/msg/task_state.hpp>
#include <nexus_orchestrator_msgs/msg/workcell_description.hpp>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <rcl_interfaces/msg/parameter_descriptor.hpp>
#include <lifecycle_msgs/msg/state.hpp>
#include <lifecycle_msgs/msg/transition.hpp>

#include <yaml-cpp/exceptions.h>

#include <chrono>
#include <filesystem>
#include <fstream>
#include <regex>
#include <sstream>
#include <string>

namespace nexus::workcell_orchestrator {

using TaskState = nexus_orchestrator_msgs::msg::TaskState;
using WorkcellRequest = endpoints::WorkcellRequestAction::ActionType;

using rcl_interfaces::msg::ParameterDescriptor;
using lifecycle_msgs::msg::State;
using lifecycle_msgs::msg::Transition;

static constexpr size_t MAX_PARALLEL_TASK = 1;
static constexpr std::chrono::milliseconds BT_TICK_RATE{10};
static constexpr std::chrono::seconds REGISTER_TICK_RATE{1};

WorkcellOrchestrator::WorkcellOrchestrator(const rclcpp::NodeOptions& options)
: rclcpp_lifecycle::LifecycleNode("workcell_orchestrator", options),
  _capability_loader("nexus_capabilities", "nexus::Capability")
{
  this->_register_workcell_client =
    this->create_client<endpoints::RegisterWorkcellService::ServiceType>(
    endpoints::RegisterWorkcellService::service_name());

  this->_register_timer = this->create_wall_timer(REGISTER_TICK_RATE,
      [this]()
      {
        this->_register();
      });

  /*
  this->_bt_timer = this->create_wall_timer(BT_TICK_RATE, [this]()
      {
        RCLCPP_INFO(this->get_logger(), "HELLO");
      });
    */
}

auto WorkcellOrchestrator::on_configure(
  const rclcpp_lifecycle::State& previous_state) -> CallbackReturn
{
  RCLCPP_INFO(this->get_logger(), "CONFIGURING");
  this->_bt_timer = this->create_wall_timer(BT_TICK_RATE, [this]()
      {
        RCLCPP_INFO(this->get_logger(), "HELLO");
      });
  return CallbackReturn::SUCCESS;
}

auto WorkcellOrchestrator::on_activate(
  const rclcpp_lifecycle::State& /* previous_state */) -> CallbackReturn
{
  RCLCPP_INFO(this->get_logger(), "Workcell activated");
  return CallbackReturn::SUCCESS;
}

auto WorkcellOrchestrator::on_deactivate(
  const rclcpp_lifecycle::State& /* previous_state */) -> CallbackReturn
{
  return CallbackReturn::SUCCESS;
}

auto WorkcellOrchestrator::on_cleanup(
  const rclcpp_lifecycle::State& /* previous_state */) -> CallbackReturn
{
  return CallbackReturn::SUCCESS;
}

auto WorkcellOrchestrator::_configure(
  const rclcpp_lifecycle::State& /* previous_state */) -> CallbackReturn
{
  return CallbackReturn::SUCCESS;
}

void WorkcellOrchestrator::_tick_bt(const std::shared_ptr<Context>& ctx)
{
  RCLCPP_INFO(this->get_logger(), "TICKING BT");
}

void WorkcellOrchestrator::_tick_all_bts()
{
  RCLCPP_INFO(this->get_logger(), "TICKING ALL BTS");
}

void WorkcellOrchestrator::_cancel_all_tasks()
{
}

void WorkcellOrchestrator::_register()
{
  if (this->_ongoing_register)
  {
    RCLCPP_ERROR(
      this->get_logger(),
      "Failed to register: No response from system orchestrator.");
    if (!this->_register_workcell_client->remove_pending_request(*this->
      _ongoing_register))
    {
      RCLCPP_WARN(this->get_logger(),
        "Unable to remove pending request during workcell registration.");
    }
  }

  RCLCPP_INFO(this->get_logger(), "Registering with system orchestrator");
  auto register_cb =
    [this](rclcpp::Client<endpoints::RegisterWorkcellService::ServiceType>::
      SharedFuture future)
    {
      this->_ongoing_register = std::nullopt;
      auto resp = future.get();
      if (!resp->success)
      {
        switch (resp->error_code)
        {
          case endpoints::RegisterWorkcellService::ServiceType::Response::
            ERROR_NOT_READY:
            RCLCPP_ERROR(
              this->get_logger(),
              "Error while registering with system orchestrator, retrying again... [%s]",
              resp->message.c_str());
            break;
          default:
            RCLCPP_FATAL(this->get_logger(),
              "Failed to register with system orchestrator! [%s]",
              resp->message.c_str());
            throw RegistrationError(resp->message, resp->error_code);
        }
        return;
      }
      RCLCPP_INFO(this->get_logger(),
        "Successfully registered with system orchestrator");
      this->_register_timer->cancel();
      this->_register_timer.reset();
    };

  if (!this->_register_workcell_client->wait_for_service(
      std::chrono::seconds{0}))
  {
    std::string msg = "Could not find system orchestrator";
    auto secs = std::chrono::seconds(REGISTER_TICK_RATE).count();
    RCLCPP_ERROR(
      this->get_logger(), "Failed to register [%s], retrying in %ld secs",
      msg.c_str(), secs);
    // timer is not canceled so it will run again.
    return;
  }

  auto req =
    std::make_shared<endpoints::RegisterWorkcellService::ServiceType::Request>();
  std::vector<std::string> caps;
  caps.reserve(this->_capabilities.size());
  for (const auto& [k, _] : this->_capabilities)
  {
    caps.emplace_back(k);
  }
  req->description.capabilities = caps;
  req->description.workcell_id = this->get_name();
  this->_ongoing_register = this->_register_workcell_client->async_send_request(
    req,
    std::move(register_cb));
}

void WorkcellOrchestrator::_process_signal(
  endpoints::SignalWorkcellService::ServiceType::Request::ConstSharedPtr req,
  endpoints::SignalWorkcellService::ServiceType::Response::SharedPtr resp)
{
}

BT::Tree WorkcellOrchestrator::_create_bt(const std::shared_ptr<Context>& ctx)
{
}

void WorkcellOrchestrator::_handle_command_success(
  const std::shared_ptr<Context>& ctx)
{
}

void WorkcellOrchestrator::_handle_command_failed(
  const std::shared_ptr<Context>& ctx)
{
}

void WorkcellOrchestrator::_handle_task_doable(
  endpoints::IsTaskDoableService::ServiceType::Request::ConstSharedPtr req,
  endpoints::IsTaskDoableService::ServiceType::Response::SharedPtr resp)
{
}

bool WorkcellOrchestrator::_can_perform_task(const Task& task)
{
  try
  {
    this->_bt_store.get_bt(task.type);
    return true;
  }
  catch (const std::out_of_range&)
  {
    return false;
  }
}

}

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(
  nexus::workcell_orchestrator::WorkcellOrchestrator)
