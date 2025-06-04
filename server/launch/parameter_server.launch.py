# Copyright 2019 Sony Corporation
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

"""Launch a server."""

from launch import LaunchDescription
from launch.substitutions import EnvironmentVariable
import launch_ros.actions
import os
import pathlib

parameters_file_name = 'parameters_via_launch.yaml'

def generate_launch_description():
    parameters_file_path = str(pathlib.Path(__file__).parents[1]) # get current path and go one level up
    parameters_file_path += '/param/' + parameters_file_name
    return LaunchDescription(
        [
            launch_ros.actions.Node(
                package="parameter_server",
                executable="server",
                output="screen",
                # respawn in 5.0 seconds
                respawn=True,
                respawn_delay=5.0,
                # these parameters in parameters_file_path cannot be registered as persistent parameters,
                # these will be loaded as normal parameter without event on /parameter_events topic.
                parameters=[parameters_file_path],
                # this example to load persistent parameter files into parameter server,
                # these parameters described in parameter_server.yaml with prefix "persistent" will be registered as persistent parameter.
                # arguments=[
                #     "--file-path",
                #     "/tmp/parameter_server.yaml",
                #     "--allow-declare",
                #     "true",
                #     "--allow-override",
                #     "true",
                #     "--storing-period",
                #     "60",
                # ],
            )
        ]
    )
