# Copyright 2026 Chris Tomaszewski
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
Bring up the sbg driver against a live device.

Launches the lifecycle node from a parameter file and, unless auto_activate:=false, drives it
configure -> activate so topics appear as soon as the device is reachable::

    ros2 launch sbg_driver bringup.launch.py params_file:=/path/to/params.yaml

The default parameter file is the installed config/serial.example.yaml (a live-device serial
config); copy it, fill in your device's settings, and pass your copy via params_file:=.

The auto-activate handler fires exactly once, on the configuring->inactive transition. If
activation fails (device unreachable) the node STAYS inactive — fix the config/cabling and
`ros2 lifecycle set <node> activate` (or restart the launch). A deliberate deactivate is
likewise not overridden.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, EmitEvent, RegisterEventHandler
from launch.conditions import IfCondition
from launch.events import matches_action
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import LifecycleNode
from launch_ros.event_handlers import OnStateTransition
from launch_ros.events.lifecycle import ChangeState
from launch_ros.substitutions import FindPackageShare
import lifecycle_msgs.msg


def generate_launch_description():
    default_params = PathJoinSubstitution(
        [FindPackageShare('sbg_driver'), 'config', 'serial.example.yaml']
    )
    params_file = LaunchConfiguration('params_file')
    auto_activate = LaunchConfiguration('auto_activate')
    namespace = LaunchConfiguration('namespace')

    # namespace='' standalone; rig pushes /<sensor-name> per instance for multi-sensor isolation.
    # The lifecycle matchers below bind to this `driver` action object (matches_action /
    # target_lifecycle_node), so the configure->activate transitions follow the node regardless of
    # the namespace it is pushed under.
    driver = LifecycleNode(
        package='sbg_driver',
        executable='sbg_driver_node',
        name='sbg_driver',
        namespace=namespace,
        output='screen',
        parameters=[params_file],
    )

    # Configure on start, then activate once configure SUCCEEDS. The start_state filter makes
    # this a one-shot: it matches only configuring->inactive — NOT activating->inactive (a failed
    # activate must not retry in an unbounded zero-backoff loop) and NOT deactivating->inactive
    # (a deliberate `lifecycle set deactivate` must not be instantly overridden). A failed device
    # open therefore leaves the node 'inactive' for the operator (or a supervisor) to retry.
    configure = EmitEvent(
        event=ChangeState(
            lifecycle_node_matcher=matches_action(driver),
            transition_id=lifecycle_msgs.msg.Transition.TRANSITION_CONFIGURE,
        ),
        condition=IfCondition(auto_activate),
    )
    activate_once_configured = RegisterEventHandler(
        OnStateTransition(
            target_lifecycle_node=driver,
            start_state='configuring',
            goal_state='inactive',
            entities=[EmitEvent(event=ChangeState(
                lifecycle_node_matcher=matches_action(driver),
                transition_id=lifecycle_msgs.msg.Transition.TRANSITION_ACTIVATE,
            ))],
        ),
        condition=IfCondition(auto_activate),
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'params_file',
            default_value=default_params,
            description='Parameter YAML (transport + frames + topics).',
        ),
        DeclareLaunchArgument(
            'auto_activate',
            default_value='true',
            description='Drive configure -> activate automatically on start.',
        ),
        DeclareLaunchArgument(
            'namespace',
            default_value='',
            description='ROS namespace to push the node under (rig sets this per sensor).',
        ),
        driver,
        activate_once_configured,
        configure,
    ])
