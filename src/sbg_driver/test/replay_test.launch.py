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
launch_testing for the file-replay path.

Phase 2 scope: bring up the lifecycle node with an empty replay file, drive
configure -> activate, assert /imu/data publisher exists and the node ends
up in the ACTIVE state. No actual data is published (empty .bin file) - that
test lands once we have captured frames in phase 3+.
"""

import os
import tempfile
import time
import unittest

import launch
from launch.actions import EmitEvent, RegisterEventHandler
from launch.event_handlers import OnProcessStart
from launch.events import matches_action
import launch_ros.actions
from launch_ros.events.lifecycle import ChangeState
import launch_testing
import launch_testing.actions
import lifecycle_msgs.msg
import lifecycle_msgs.srv
import pytest
import rclpy


@pytest.mark.launch_test
def generate_test_description():
    # An empty file is a valid sbgInterfaceFile target - the SDK opens it
    # cleanly and immediately hits EOF without delivering any frames.
    fd, replay_path = tempfile.mkstemp(suffix='.bin', prefix='sbg_replay_')
    os.close(fd)

    driver = launch_ros.actions.LifecycleNode(
        package='sbg_driver',
        executable='sbg_driver_node',
        name='sbg_driver',
        namespace='',
        output='screen',
        parameters=[
            {
                'transport.type': 'file',
                'transport.file.path': replay_path,
                'transport.file.real_time_pace': False,
                'frames.imu': 'imu_link',
                'convention.use_enu': False,
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
    drive = RegisterEventHandler(
        OnProcessStart(target_action=driver, on_start=[configure, activate])
    )

    return (
        launch.LaunchDescription(
            [driver, drive, launch_testing.actions.ReadyToTest()]
        ),
        {'driver': driver, 'replay_path': replay_path},
    )


class TestDriverLifecycle(unittest.TestCase):

    @classmethod
    def setUpClass(cls):
        rclpy.init()
        cls.node = rclpy.create_node('replay_test_client')

    @classmethod
    def tearDownClass(cls):
        cls.node.destroy_node()
        rclpy.shutdown()

    def _get_state(self, node_name: str = 'sbg_driver', timeout: float = 5.0):
        client = self.node.create_client(
            lifecycle_msgs.srv.GetState, f'/{node_name}/get_state'
        )
        self.assertTrue(
            client.wait_for_service(timeout_sec=timeout),
            f'/{node_name}/get_state did not appear',
        )
        future = client.call_async(lifecycle_msgs.srv.GetState.Request())
        rclpy.spin_until_future_complete(self.node, future, timeout_sec=timeout)
        self.assertTrue(future.done(), 'GetState call did not complete')
        return future.result().current_state.id

    def test_node_reaches_active_state(self):
        # Give the lifecycle a moment to drive configure -> activate.
        deadline = time.time() + 10.0
        state = None
        target = lifecycle_msgs.msg.State.PRIMARY_STATE_ACTIVE
        while time.time() < deadline:
            try:
                state = self._get_state()
                if state == target:
                    break
            except Exception:
                pass
            time.sleep(0.5)
        self.assertEqual(
            state, target,
            f'node did not reach ACTIVE (state={state})'
        )

    def test_imu_topic_exists(self):
        # Give discovery a beat.
        time.sleep(2.0)
        topics = self.node.get_topic_names_and_types()
        names = {n for n, _ in topics}
        self.assertIn(
            '/imu/data', names,
            f'/imu/data missing from topic list: {names}',
        )


@launch_testing.post_shutdown_test()
class TestProcessExit(unittest.TestCase):

    def test_exit_code_ok(self, proc_info, driver):
        launch_testing.asserts.assertExitCodes(proc_info, process=driver)
