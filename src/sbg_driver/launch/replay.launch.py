# Copyright 2026 sbg_driver maintainers
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

"""
Launch the SBG driver in file-replay mode.

Usage:
    ros2 launch sbg_driver replay.launch.py bag:=/path/to/sample.bin

The launch file brings up the node in `unconfigured`, then drives it through
`configure -> active` automatically so a consumer subscribing to /imu/data
sees output as soon as the node is up.
"""

import launch
from launch.actions import DeclareLaunchArgument, EmitEvent, RegisterEventHandler
from launch.event_handlers import OnProcessStart
from launch.events import matches_action
from launch.substitutions import LaunchConfiguration
import launch_ros.actions
from launch_ros.events.lifecycle import ChangeState
import lifecycle_msgs.msg


def generate_launch_description() -> launch.LaunchDescription:
    bag_arg = DeclareLaunchArgument(
        'bag',
        default_value='',
        description='Path to .bin replay file',
    )
    frame_arg = DeclareLaunchArgument(
        'frame_id',
        default_value='imu_link',
        description='TF frame ID for IMU outputs',
    )
    use_enu_arg = DeclareLaunchArgument(
        'use_enu',
        default_value='false',
        description='Convert sensor-native NED outputs to REP-103 ENU',
    )

    driver = launch_ros.actions.LifecycleNode(
        package='sbg_driver',
        executable='sbg_driver_node',
        name='sbg_driver',
        namespace='',
        output='screen',
        parameters=[
            {
                'transport.type': 'file',
                'transport.file.path': LaunchConfiguration('bag'),
                'transport.file.real_time_pace': True,
                'frames.imu': LaunchConfiguration('frame_id'),
                'convention.use_enu': LaunchConfiguration('use_enu'),
            }
        ],
    )

    configure = EmitEvent(
        event=ChangeState(
            lifecycle_node_matcher=matches_action(driver),
            transition_id=lifecycle_msgs.msg.Transition.TRANSITION_CONFIGURE,
        )
    )
    activate = EmitEvent(
        event=ChangeState(
            lifecycle_node_matcher=matches_action(driver),
            transition_id=lifecycle_msgs.msg.Transition.TRANSITION_ACTIVATE,
        )
    )

    drive_lifecycle = RegisterEventHandler(
        OnProcessStart(target_action=driver, on_start=[configure, activate])
    )

    return launch.LaunchDescription(
        [bag_arg, frame_arg, use_enu_arg, driver, drive_lifecycle]
    )
