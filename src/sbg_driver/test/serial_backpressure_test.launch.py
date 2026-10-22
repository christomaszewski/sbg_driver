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

"""Exercise lifecycle, commands and signal shutdown with a blocked real PTY."""

import os
import pty
import signal
import time
import unittest

import launch
import launch_ros.actions
import launch_testing
import launch_testing.actions
import lifecycle_msgs.srv
import pytest
import rclpy
from rtcm_msgs.msg import Message
from std_srvs.srv import Trigger


@pytest.mark.launch_test
def generate_test_description():
    master, slave = pty.openpty()
    driver = launch_ros.actions.LifecycleNode(
        package='sbg_driver', executable='sbg_driver_node', name='sbg_driver',
        namespace='serial_stall', output='screen', parameters=[{
            'transport.type': 'serial',
            'transport.serial.port': os.ttyname(slave),
            'health.publish_interval_s': 0.1,
        }])
    return (
        launch.LaunchDescription([driver, launch_testing.actions.ReadyToTest()]),
        {'driver': driver, 'serial_fds': (master, slave)},
    )


class TestSerialBackpressure(unittest.TestCase):

    @classmethod
    def setUpClass(cls):
        rclpy.init()
        cls.node = rclpy.create_node('serial_backpressure_test')

    @classmethod
    def tearDownClass(cls):
        cls.node.destroy_node()
        rclpy.shutdown()

    def call(self, client, request, timeout=3.0):
        self.assertTrue(client.wait_for_service(timeout_sec=10.0))
        future = client.call_async(request)
        rclpy.spin_until_future_complete(self.node, future, timeout_sec=timeout)
        self.assertTrue(future.done(), 'service stalled under serial backpressure')
        return future.result()

    def test_stalled_serial_keeps_control_responsive(self, driver, proc_info):
        change = self.node.create_client(
            lifecycle_msgs.srv.ChangeState, '/serial_stall/sbg_driver/change_state')
        state = self.node.create_client(
            lifecycle_msgs.srv.GetState, '/serial_stall/sbg_driver/get_state')
        calibration = self.node.create_client(Trigger, '/serial_stall/sbg/start_mag_calibration')
        publisher = self.node.create_publisher(Message, '/serial_stall/rtcm', 20)

        def transition(identifier):
            request = lifecycle_msgs.srv.ChangeState.Request()
            request.transition.id = identifier
            self.assertTrue(self.call(change, request, timeout=10.0).success)

        def flood():
            # The master is intentionally never read. Exceed the kernel PTY
            # queue, then exercise controls while correction writes stall.
            message = Message()
            message.message = [0] * 1024
            for _ in range(80):
                publisher.publish(message)
                time.sleep(0.01)

        transition(1)  # configure
        transition(3)  # activate (no INFO responder)
        deadline = time.monotonic() + 5
        while publisher.get_subscription_count() == 0 and time.monotonic() < deadline:
            rclpy.spin_once(self.node, timeout_sec=0.05)
        self.assertGreater(publisher.get_subscription_count(), 0)
        flood()
        self.call(state, lifecycle_msgs.srv.GetState.Request())
        # Pauses and joins the writer before SDK request/response, then
        # resumes it when the deliberately absent command ACK times out.
        self.assertFalse(self.call(calibration, Trigger.Request(), timeout=10.0).success)
        flood()
        start = time.monotonic()
        transition(4)  # deactivate: active write is cancelled, queue discarded
        self.assertLess(time.monotonic() - start, 3.0)
        transition(3)  # new device / workers
        flood()
        self.call(state, lifecycle_msgs.srv.GetState.Request())
        os.kill(driver.process_details['pid'], signal.SIGTERM)
        proc_info.assertWaitForShutdown(process=driver, timeout=5)


@launch_testing.post_shutdown_test()
class TestSerialBackpressureExit(unittest.TestCase):

    def test_clean_exit(self, proc_info, driver, serial_fds):
        try:
            launch_testing.asserts.assertExitCodes(proc_info, process=driver)
        finally:
            for descriptor in serial_fds:
                os.close(descriptor)
