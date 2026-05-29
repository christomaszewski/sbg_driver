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

Brings up the lifecycle node with an empty replay file, then drives
configure -> activate *deterministically via change_state service calls*
(each awaited before the next), and asserts the node reaches ACTIVE and
publishes /imu/data. Driving the transitions from the test body — rather
than firing two EmitEvents on process start — avoids the race where
`activate` arrives before `configure` has finished (which previously made
this test flake under colcon).

An empty .bin file is a valid sbgInterfaceFile target: the SDK opens it and
immediately hits EOF, so no data is published — this exercises the lifecycle
and topic-graph wiring, not data flow.
"""

import os
import tempfile
import unittest

import launch
import launch_ros.actions
import launch_testing
import launch_testing.actions
import lifecycle_msgs.msg
import lifecycle_msgs.srv
import pytest
import rclpy


@pytest.mark.launch_test
def generate_test_description():
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

    return (
        launch.LaunchDescription([driver, launch_testing.actions.ReadyToTest()]),
        {'driver': driver, 'replay_path': replay_path},
    )


class TestDriverLifecycle(unittest.TestCase):

    @classmethod
    def setUpClass(cls):
        rclpy.init()
        cls.node = rclpy.create_node('replay_test_client')
        cls.change_client = cls.node.create_client(
            lifecycle_msgs.srv.ChangeState, '/sbg_driver/change_state'
        )
        cls.get_client = cls.node.create_client(
            lifecycle_msgs.srv.GetState, '/sbg_driver/get_state'
        )
        # The node process needs a few seconds to start and expose its
        # lifecycle services; CI containers are slower than a dev box.
        assert cls.change_client.wait_for_service(timeout_sec=20.0), \
            '/sbg_driver/change_state did not appear'
        assert cls.get_client.wait_for_service(timeout_sec=20.0), \
            '/sbg_driver/get_state did not appear'

        # Drive configure -> activate, awaiting each transition's result so
        # they never race.
        assert cls._change_state(lifecycle_msgs.msg.Transition.TRANSITION_CONFIGURE), \
            'configure transition was rejected'
        assert cls._change_state(lifecycle_msgs.msg.Transition.TRANSITION_ACTIVATE), \
            'activate transition was rejected'

    @classmethod
    def tearDownClass(cls):
        cls.node.destroy_node()
        rclpy.shutdown()

    @classmethod
    def _change_state(cls, transition_id: int, timeout: float = 15.0) -> bool:
        req = lifecycle_msgs.srv.ChangeState.Request()
        req.transition.id = transition_id
        future = cls.change_client.call_async(req)
        rclpy.spin_until_future_complete(cls.node, future, timeout_sec=timeout)
        return future.done() and future.result().success

    def _current_state(self, timeout: float = 10.0) -> int:
        future = self.get_client.call_async(lifecycle_msgs.srv.GetState.Request())
        rclpy.spin_until_future_complete(self.node, future, timeout_sec=timeout)
        self.assertTrue(future.done(), 'GetState call did not complete')
        return future.result().current_state.id

    def test_node_is_active(self):
        self.assertEqual(
            self._current_state(),
            lifecycle_msgs.msg.State.PRIMARY_STATE_ACTIVE,
            'node is not in the ACTIVE state after configure + activate',
        )

    def test_imu_topic_exists(self):
        # Spin briefly so topic-graph discovery settles.
        end = self.node.get_clock().now().nanoseconds + 3_000_000_000
        while self.node.get_clock().now().nanoseconds < end:
            rclpy.spin_once(self.node, timeout_sec=0.2)
            names = {n for n, _ in self.node.get_topic_names_and_types()}
            if '/imu/data' in names:
                break
        names = {n for n, _ in self.node.get_topic_names_and_types()}
        self.assertIn('/imu/data', names, f'/imu/data missing from: {sorted(names)}')


@launch_testing.post_shutdown_test()
class TestProcessExit(unittest.TestCase):

    def test_exit_code_ok(self, proc_info, driver):
        launch_testing.asserts.assertExitCodes(proc_info, process=driver)
