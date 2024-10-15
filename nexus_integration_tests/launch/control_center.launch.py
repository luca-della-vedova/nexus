# Copyright (C) 2022 Johnson & Johnson
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import os
from ament_index_python.packages import get_package_share_directory

import launch_ros
from launch_ros.actions import Node, LifecycleNode
from launch_ros.event_handlers import OnStateTransition
from launch_ros.substitutions import FindPackageShare
import lifecycle_msgs

import launch
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    GroupAction,
    IncludeLaunchDescription,
    LogInfo,
    OpaqueFunction,
    RegisterEventHandler,
    SetEnvironmentVariable,
)
from launch.conditions import IfCondition, UnlessCondition
from launch.event_handlers import OnProcessExit
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution


def activate_node_service(node_name, ros_domain_id):
    activate_node_proc = ExecuteProcess(
        cmd=[
            'python3',
            [FindPackageShare('nexus_integration_tests'), "/scripts/activate_node.py"],
            node_name,
        ],
        additional_env={'ROS_DOMAIN_ID': ros_domain_id},
    )

    def check_activate_return_code(event, _):
        if event.returncode != 0:
            return [
                LogInfo(msg=f"Activating node '{node_name}' failed!"),
                launch.actions.EmitEvent(event=Shutdown(reason=f"Activating node '{node_name}' failed!"))
            ]
        return []

    return GroupAction(
        [
            LogInfo(msg=f"Activating {node_name}..."),
            activate_node_proc,
            RegisterEventHandler(
                OnProcessExit(
                    target_action=activate_node_proc,
                    on_exit=check_activate_return_code,
                )
            ),
        ],
    )

def activate_node(target_node: LifecycleNode, depend_node: LifecycleNode = None):

    configure_event = None

    if depend_node is not None:
        configure_event = launch.actions.RegisterEventHandler(
            OnStateTransition(
                target_lifecycle_node=depend_node,
                goal_state="active",
                entities=[
                    launch.actions.EmitEvent(
                        event=launch_ros.events.lifecycle.ChangeState(
                            lifecycle_node_matcher=launch.events.matches_action(
                                target_node
                            ),
                            transition_id=lifecycle_msgs.msg.Transition.TRANSITION_CONFIGURE,
                        )
                    )
                ],
            )
        )
    else:
        # Make the talker node take the 'configure' transition.
        configure_event = launch.actions.RegisterEventHandler(
            OnStateTransition(
                target_lifecycle_node=target_node,
                goal_state="unconfigured",
                entities=[
                    launch.actions.EmitEvent(
                        event=launch_ros.events.lifecycle.ChangeState(
                            lifecycle_node_matcher=launch.events.matches_action(target_node),
                            transition_id=lifecycle_msgs.msg.Transition.TRANSITION_CONFIGURE,
                        )
                    )
                ],
            )
        )

    inactive_state_handler = launch.actions.RegisterEventHandler(
        OnStateTransition(
            target_lifecycle_node=target_node,
            goal_state="inactive",
            entities=[
                launch.actions.LogInfo(
                    msg=f"node {target_node.node_executable} reached the 'inactive' state."
                ),
                launch.actions.EmitEvent(
                    event=launch_ros.events.lifecycle.ChangeState(
                        lifecycle_node_matcher=launch.events.matches_action(
                            target_node
                        ),
                        transition_id=lifecycle_msgs.msg.Transition.TRANSITION_ACTIVATE,
                    )
                ),
            ],
        )
    )

    active_state_handler = launch.actions.RegisterEventHandler(
        OnStateTransition(
            target_lifecycle_node=target_node,
            goal_state="active",
            entities=[
                launch.actions.LogInfo(
                    msg=f"node {target_node.node_executable} reached the 'active' state"
                ),
            ],
        )
    )

    node_name = target_node.node_executable
    dbg = launch.actions.LogInfo(msg=f"CONFIGURING NODE {node_name}")

    return GroupAction(
        [
            dbg,
            inactive_state_handler,
            active_state_handler,
            configure_event,
        ]
    )


def launch_setup(context, *args, **kwargs):
    ros_domain_id = LaunchConfiguration("ros_domain_id")
    use_zenoh_bridge = LaunchConfiguration("use_zenoh_bridge")
    zenoh_config_package = LaunchConfiguration("zenoh_config_package")
    zenoh_config_filename = LaunchConfiguration("zenoh_config_filename")
    transporter_plugin = LaunchConfiguration("transporter_plugin")
    activate_system_orchestrator = LaunchConfiguration("activate_system_orchestrator")
    headless = LaunchConfiguration("headless")

    nexus_panel_rviz_path = os.path.join(
        get_package_share_directory("nexus_integration_tests"), "rviz", "nexus_panel.rviz"
    )

    system_orchestrator_node = LifecycleNode(
        name="system_orchestrator",
        namespace="",
        package="nexus_system_orchestrator",
        executable="nexus_system_orchestrator",
        parameters=[
            {
                "bt_path": (
                    FindPackageShare("nexus_integration_tests"),
                    "/config/system_bts",
                ),
                "remap_task_types":
                    """{
                        pick_and_place: [place_on_conveyor, pick_from_conveyor],
                    }""",
                "max_jobs": 2,
            }
        ],
    )

    transporter_node = LifecycleNode(
        namespace="",
        package="nexus_transporter",
        executable="nexus_transporter_node",
        name="transporter_node",
        parameters=[
            {"transporter_plugin": transporter_plugin},
            {"destinations": ["loading", "unloading", "workcell_1", "workcell_2"]},
            {"x_increment": 1.0},
            {"speed": 1.0},
            {"unloading_station": "unloading"},
        ],
    )

    mock_emergency_alarm_node = LifecycleNode(
        name="mock_emergency_alarm",
        namespace="",
        package="nexus_integration_tests",
        executable="mock_emergency_alarm",
    )

    nexus_panel = Node(
        package="rviz2",
        executable="rviz2",
        name="nexus_panel",
        arguments=["-d", nexus_panel_rviz_path],
        condition=UnlessCondition(headless),
    )

    return [
        SetEnvironmentVariable("ROS_DOMAIN_ID", ros_domain_id),
        system_orchestrator_node,
        transporter_node,
        mock_emergency_alarm_node,
        GroupAction(
            [
                IncludeLaunchDescription(
                    [
                        PathJoinSubstitution(
                            [
                                FindPackageShare("nexus_integration_tests"),
                                "launch",
                                "zenoh_bridge.launch.py",
                            ]
                        )
                    ],
                    launch_arguments={
                        "zenoh_config_package": zenoh_config_package,
                        "zenoh_config_filename": zenoh_config_filename,
                        "ros_domain_id": ros_domain_id.perform(context),
                    }.items(),
                )
            ],
            condition=IfCondition(use_zenoh_bridge),
        ),
        activate_node_service("system_orchestrator", ros_domain_id),
        activate_node_service("transporter_node", ros_domain_id),
        activate_node_service("mock_emergency_alarm", ros_domain_id),
        #GroupAction(
        #    [
        #        activate_node(system_orchestrator_node),
        #    ],
        #    condition=IfCondition(activate_system_orchestrator),
        #),
        #activate_node(transporter_node),
        #activate_node(mock_emergency_alarm_node),
        nexus_panel,
    ]


def generate_launch_description():

    return launch.LaunchDescription(
        [
            DeclareLaunchArgument(
                "ros_domain_id",
                default_value="0",
                description="ROS_DOMAIN_ID environment variable",
            ),
            DeclareLaunchArgument(
                "use_zenoh_bridge",
                default_value="true",
                description="Set true to launch the Zenoh DDS Bridge",
            ),
            DeclareLaunchArgument(
                name="zenoh_config_package",
                default_value="nexus_integration_tests",
                description="Package containing Zenoh DDS bridge configurations",
            ),
            DeclareLaunchArgument(
                name="zenoh_config_filename",
                default_value="config/zenoh/system_orchestrator.json5",
                description="Zenoh DDS bridge configuration filepath",
            ),
            DeclareLaunchArgument(
                "transporter_plugin",
                default_value="nexus_transporter::MockTransporter",
                description="The transporter plugin to load by the TransporterNode",
            ),
            DeclareLaunchArgument(
                "activate_system_orchestrator",
                default_value="false",
                description="Whether to activate system orchestrator upon launch",
            ),
            DeclareLaunchArgument(
                "headless",
                default_value="true",
                description="Launch in headless mode (no gui)",
            ),
            OpaqueFunction(function=launch_setup),
        ]
    )
