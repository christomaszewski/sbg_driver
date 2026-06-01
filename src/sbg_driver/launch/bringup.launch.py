# Copyright 2026 Chris Tomaszewski
#
# Licensed under the Apache License, Version 2.0 (the "License"); see LICENSE.

"""
Bring up the sbg driver against a live device.

Launches the lifecycle node from a parameter file and, unless auto_activate:=false, drives it
configure -> activate so topics appear as soon as the device is reachable::

    ros2 launch sbg_driver bringup.launch.py params_file:=/path/to/params.yaml

The default parameter file is the installed config/replay.example.yaml; copy it, fill in
your device's settings, and pass your copy via params_file:=.
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
        [FindPackageShare('sbg_driver'), 'config', 'replay.example.yaml']
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

    # Configure on start, then activate once the node reports 'inactive'. Driving the transitions in
    # order avoids the activate-before-configure race; a failed device open leaves the node
    # 'inactive' instead of crashing.
    configure = EmitEvent(
        event=ChangeState(
            lifecycle_node_matcher=matches_action(driver),
            transition_id=lifecycle_msgs.msg.Transition.TRANSITION_CONFIGURE,
        ),
        condition=IfCondition(auto_activate),
    )
    activate_on_inactive = RegisterEventHandler(
        OnStateTransition(
            target_lifecycle_node=driver,
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
        activate_on_inactive,
        configure,
    ])
