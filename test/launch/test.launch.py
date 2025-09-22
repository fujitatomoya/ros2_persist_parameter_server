# Copyright 2020 Sony Corporation
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

"""Launch server && client"""

from launch import LaunchDescription
from launch.substitutions import EnvironmentVariable
from launch.actions import ExecuteProcess, DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    allow_dynamic_typing_arg = DeclareLaunchArgument(
        'allow_dynamic_typing', default_value='false', description='Enable dynamic typing for parameters'
    )

    return LaunchDescription([
        allow_dynamic_typing_arg,
        ExecuteProcess(
            cmd=[
                'ros2', 'run', 'parameter_server', 'server',
                '--file-path', '/tmp/test/parameter_server.yaml',
                '--allow-dynamic-typing', LaunchConfiguration(
                    'allow_dynamic_typing')
            ],
            respawn=True
        )
    ])
