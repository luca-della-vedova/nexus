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

from launch_ros.actions import LifecycleNode
from launch_ros.parameter_descriptions import Parameter
from launch_ros.substitutions import FindPackageShare

import launch
from launch.actions import (
    DeclareLaunchArgument,
    EmitEvent,
    ExecuteProcess,
    GroupAction,
    IncludeLaunchDescription,
    LogInfo,
    OpaqueFunction,
    RegisterEventHandler,
    SetEnvironmentVariable,
)
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, FindExecutable

def launch_setup(context, *args, **kwargs):
    workcell_orchestrator_node = LifecycleNode(
        name="workcell_1",
        namespace="",
        package="nexus_workcell_orchestrator",
        executable="nexus_workcell_orchestrator",
    )

    return [
        workcell_orchestrator_node,
    ]


def generate_launch_description():

    return launch.LaunchDescription([
        OpaqueFunction(function = launch_setup)
    ])
