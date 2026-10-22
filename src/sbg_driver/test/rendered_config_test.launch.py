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

"""Exercise rig-rendered and shipped ROS YAML through real node startup."""

import copy
import os
from pathlib import Path
import subprocess
import sys
import tempfile
import unittest

import launch
import launch_ros.actions
import launch_testing
import launch_testing.actions
import launch_testing.asserts
import lifecycle_msgs.msg
import lifecycle_msgs.srv
import pytest
import rcl_interfaces.msg
import rcl_interfaces.srv
import rclpy
import yaml


@pytest.mark.launch_test
def generate_test_description():
    root = Path(__file__).resolve().parents[3]
    staging = tempfile.TemporaryDirectory(prefix='sbg_rendered_config_')
    directory = Path(staging.name)
    replay = directory / 'empty.bin'
    replay.touch()
    example = yaml.safe_load((root / 'sensors/sbg.example.yaml').read_text())
    cases = []

    for name, generic, override, expected in (
        ('example', [], {}, []),
        ('omitted', None, {}, []),
        ('nonempty', ['CPU', 'DATALOGGER'], {}, ['CPU', 'DATALOGGER']),
        ('nested_clear', ['CPU'], {'health': {'ignored_alerts': []}}, []),
        ('flat_clear', ['CPU'], {'health.ignored_alerts': []}, []),
        ('nested_value', [], {'health': {'ignored_alerts': ['CPU']}}, ['CPU']),
        ('flat_value', ['DATALOGGER'], {'health.ignored_alerts': ['CPU']}, ['CPU']),
        ('output_rates', [], {'outputs': {'rates_hz': {
            'imu_data': 200.0, 'odom': 200.0,
            'imu_temperature': 1.0, 'sbg_ekf_status': 1.0,
        }}}, []),
    ):
        config = copy.deepcopy(example)
        config['name'] = name
        config['ros']['namespace'] = name
        config['driver_params'] = override
        if generic is None:
            config['health'].pop('ignored_alerts')
        else:
            config['health']['ignored_alerts'] = generic
        source = directory / f'{name}.yaml'
        source.write_text(yaml.safe_dump(config))
        rendered = subprocess.run(
            [sys.executable, str(root / 'tools/render_params.py'), '--container', str(source)],
            check=True, capture_output=True, text=True,
            env=dict(os.environ, VEHICLE_ID='test_vehicle'))
        params = directory / f'{name}.params.yaml'
        params.write_text(rendered.stdout)
        cases.append((name, params, expected))

    for kind in ('serial', 'replay'):
        name = f'native_{kind}'
        source = root / 'src/sbg_driver/config' / f'{kind}.example.yaml'
        # Only qualify the test node's namespace; retain every shipped value.
        config = yaml.safe_load(source.read_text())
        params = directory / f'{name}.params.yaml'
        params.write_text(yaml.safe_dump({f'/{name}/sbg_driver': config['sbg_driver']}))
        cases.append((name, params, []))

    drivers = []
    for name, params, _ in cases:
        drivers.append(launch_ros.actions.LifecycleNode(
            package='sbg_driver', executable='sbg_driver_node', name='sbg_driver',
            namespace=name, output='screen', parameters=[str(params), {
                'transport.type': 'file',
                'transport.file.path': str(replay),
                'transport.file.real_time_pace': False,
            }]))
    return (
        launch.LaunchDescription([*drivers, launch_testing.actions.ReadyToTest()]),
        {'cases': cases, 'drivers': drivers, 'staging': staging},
    )


class TestRenderedConfig(unittest.TestCase):

    @classmethod
    def setUpClass(cls):
        rclpy.init()
        cls.node = rclpy.create_node('rendered_config_test_client')

    @classmethod
    def tearDownClass(cls):
        cls.node.destroy_node()
        rclpy.shutdown()

    def call(self, client, request):
        self.assertTrue(client.wait_for_service(timeout_sec=20.0), client.srv_name)
        future = client.call_async(request)
        rclpy.spin_until_future_complete(self.node, future, timeout_sec=10.0)
        self.assertTrue(future.done(), client.srv_name)
        self.assertIsNotNone(future.result())
        return future.result()

    def test_all_configs_start_with_typed_alert_lists(self, cases):
        for name, _, expected in cases:
            with self.subTest(config=name):
                base = f'/{name}/sbg_driver'
                change = self.node.create_client(
                    lifecycle_msgs.srv.ChangeState, base + '/change_state')
                get = self.node.create_client(
                    rcl_interfaces.srv.GetParameters, base + '/get_parameters')
                try:
                    request = lifecycle_msgs.srv.ChangeState.Request()
                    request.transition.id = lifecycle_msgs.msg.Transition.TRANSITION_CONFIGURE
                    self.assertTrue(self.call(change, request).success)
                    params = self.call(get, rcl_interfaces.srv.GetParameters.Request(
                        names=['health.ignored_alerts'])).values
                    self.assertEqual(len(params), 1)
                    self.assertEqual(
                        params[0].type,
                        rcl_interfaces.msg.ParameterType.PARAMETER_STRING_ARRAY)
                    self.assertEqual(list(params[0].string_array_value), expected)
                    if name == 'output_rates':
                        rate_names = [f'outputs.rates_hz.{key}' for key in (
                            'imu_data', 'odom', 'imu_temperature', 'sbg_ekf_status')]
                        rates = self.call(get, rcl_interfaces.srv.GetParameters.Request(
                            names=rate_names)).values
                        self.assertEqual([rate.double_value for rate in rates],
                                         [200.0, 200.0, 1.0, 1.0])
                        self.assertTrue(all(
                            rate.type == rcl_interfaces.msg.ParameterType.PARAMETER_DOUBLE
                            for rate in rates))
                        describe = self.node.create_client(
                            rcl_interfaces.srv.DescribeParameters, base + '/describe_parameters')
                        try:
                            descriptors = self.call(
                                describe, rcl_interfaces.srv.DescribeParameters.Request(
                                    names=rate_names)).descriptors
                            self.assertTrue(all(item.read_only for item in descriptors))
                        finally:
                            self.node.destroy_client(describe)
                    request.transition.id = lifecycle_msgs.msg.Transition.TRANSITION_ACTIVATE
                    self.assertTrue(self.call(change, request).success)
                    request.transition.id = (
                        lifecycle_msgs.msg.Transition.TRANSITION_ACTIVE_SHUTDOWN)
                    self.assertTrue(self.call(change, request).success)
                finally:
                    self.node.destroy_client(get)
                    self.node.destroy_client(change)


@launch_testing.post_shutdown_test()
class TestProcessExit(unittest.TestCase):

    def test_exit_codes(self, proc_info, drivers, staging):
        try:
            for driver in drivers:
                launch_testing.asserts.assertExitCodes(proc_info, process=driver)
        finally:
            staging.cleanup()
