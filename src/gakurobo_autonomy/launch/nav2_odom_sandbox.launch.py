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
import os

from ament_index_python.packages import (
    get_package_share_directory,
)

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
)
from launch.launch_description_sources import (
    PythonLaunchDescriptionSource,
)
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition


def generate_launch_description():
    use_sim_time = LaunchConfiguration("use_sim_time")
    autostart = LaunchConfiguration("autostart")
    use_rviz = LaunchConfiguration("use_rviz")

    autonomy_share = get_package_share_directory(
        "gakurobo_autonomy"
    )

    nav2_bringup_share = get_package_share_directory(
        "nav2_bringup"
    )

    default_params_file = os.path.join(
        autonomy_share,
        "config",
        "nav2_odom_sandbox.yaml",
    )
    params_file = LaunchConfiguration("params_file")

    navigation_launch = os.path.join(
        nav2_bringup_share,
        "launch",
        "navigation_launch.py",
    )

    rviz_launch = os.path.join(
        nav2_bringup_share,
        "launch",
        "rviz_launch.py",
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            "params_file",
            default_value=default_params_file,
            description="Full path to the Nav2 parameter file",
        ),

        DeclareLaunchArgument(
            "use_sim_time",
            default_value="true",
            description="Use the Gazebo simulation clock",
        ),

        DeclareLaunchArgument(
            "autostart",
            default_value="true",
            description="Activate Nav2 lifecycle nodes",
        ),

        DeclareLaunchArgument(
            "use_rviz",
            default_value="true",
            description="Start the Nav2 RViz configuration",
        ),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                navigation_launch
            ),
            launch_arguments={
                "use_sim_time": use_sim_time,
                "autostart": autostart,
                "params_file": params_file,
                "use_composition": "False",
                "use_respawn": "False",
                "namespace": "",
            }.items(),
        ),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                rviz_launch
            ),
            launch_arguments={
                "namespace": "",
                "use_namespace": "False",
                "rviz_config": os.path.join(
                    nav2_bringup_share,
                    "rviz",
                    "nav2_default_view.rviz",
                ),
                "use_sim_time": use_sim_time,
            }.items(),
            condition=IfCondition(use_rviz),
        ),
    ])
