# Copyright 2026 RoboPro2027
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
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

import os


def generate_launch_description():
    package_share = get_package_share_directory("gakurobo_autonomy")

    default_bt_xml = os.path.join(
        package_share,
        "behavior_trees",
        "demo_wait.xml",
    )

    bt_xml_argument = DeclareLaunchArgument(
        "bt_xml",
        default_value=default_bt_xml,
        description="Path to the mission Behavior Tree XML file",
    )

    mission_runner = Node(
        package="gakurobo_autonomy",
        executable="mission_runner",
        name="mission_runner",
        output="screen",
        parameters=[
            {
                "bt_xml": LaunchConfiguration("bt_xml"),
            }
        ],
    )

    return LaunchDescription(
        [
            bt_xml_argument,
            mission_runner,
        ]
    )
