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
launch_testing for fleet health on /diagnostics, end to end.

Real driver processes. ``front`` replays a capture built here from valid
sbgECom frames (STATUS, IMU_DATA, the EKF logs, GPS1_POS, GPS1_HDT), paced in real
time, so the test sees what a dashboard would: a confirmed standby, live
measurements, a device fault raised and cleared, then - when the capture ends -
the stream going silent while the last-known values turn stale. ``broken``
points at a file that does not exist, for the failed-activation report.
``stopper`` and ``terminator`` are stopped by signal, the way a container stop
or Ctrl-C ends the process, and must leave through their lifecycle shutdown.

Everything is read back through a real subscription in another process, so
the DiagnosticArray crosses the RMW and its serialization.
"""

import math
import os
import signal
import struct
import tempfile
import time
import unittest

from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus
import launch
import launch_ros.actions
import launch_testing
import launch_testing.actions
import lifecycle_msgs.msg
import lifecycle_msgs.srv
import pytest
import rclpy
from rclpy.qos import DurabilityPolicy, qos_profile_sensor_data, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Imu

# sbgECom message ids (class SBG_ECOM_CLASS_LOG_ECOM_0).
LOG_STATUS, LOG_IMU_DATA, LOG_EKF_QUAT, LOG_EKF_NAV = 1, 3, 7, 8
LOG_GPS1_POS, LOG_GPS1_HDT, LOG_EKF_VEL_BODY = 14, 15, 54

GENERAL_ALL_OK = 0x007F
GENERAL_CPU_OK = 1 << 6
# NAV_POSITION, every validity bit, GPS1 position/velocity/heading used.
EKF_NAV_POSITION = 4 | (0xF << 4) | (1 << 10) | (1 << 11) | (1 << 13)
GNSS_RTK_INT = 7 << 6  # solution status SOL_COMPUTED (0) | type RTK_INT

CAPTURE_SECONDS = 4.0


def crc16(data: bytes) -> int:
    crc = 0
    for byte in data:
        crc ^= byte
        for _ in range(8):
            crc = (crc >> 1) ^ 0x8408 if crc & 1 else crc >> 1
    return crc


def frame(msg_id: int, payload: bytes) -> bytes:
    body = struct.pack('<BBH', msg_id, 0, len(payload)) + payload
    return b'\xff\x5a' + body + struct.pack('<H', crc16(body)) + b'\x33'


def write_capture(path: str) -> None:
    """
    Write CAPTURE_SECONDS of logs: 20 Hz IMU + EKF, 2 Hz STATUS / GPS1_POS / GPS1_HDT.

    The EKF triple (quat, nav, body velocity) shares each tick's timestamp, as
    on a real unit, so /odom composes and the stream has nothing to warn about.
    """
    frames = []
    for tick in range(int(CAPTURE_SECONDS * 20)):
        ts = 1_000_000 + tick * 50_000
        seconds = tick / 20.0
        frames.append(frame(LOG_IMU_DATA, struct.pack(
            '<IH3f3ff3f3f', ts, 0x03FF, 0.0, 0.0, -9.81, 0.0, 0.0, 0.0, 37.5,
            0.0, 0.0, -9.81, 0.0, 0.0, 0.0)))
        frames.append(frame(LOG_EKF_QUAT, struct.pack(
            '<I4f3fI', ts, 1.0, 0.0, 0.0, 0.0, 0.01, 0.01, 0.02, EKF_NAV_POSITION)))
        frames.append(frame(LOG_EKF_VEL_BODY, struct.pack(
            '<II3f3f', ts, EKF_NAV_POSITION, 0.0, 0.0, 0.0, 0.01, 0.01, 0.01)))
        frames.append(frame(LOG_EKF_NAV, struct.pack(
            '<I3f3f3df3fI', ts, 0.0, 0.0, 0.0, 0.01, 0.01, 0.01, 46.2, 6.1, 400.0, 48.0,
            0.02, 0.02, 0.03, EKF_NAV_POSITION)))
        if tick % 10 == 0:
            # CPU headroom fault for the first second, then cleared.
            general = GENERAL_ALL_OK & ~GENERAL_CPU_OK if seconds < 1.0 else GENERAL_ALL_OK
            frames.append(frame(LOG_STATUS, struct.pack(
                '<IHHIIIHIB', ts, general, 0x0003, 0x0000801F, 0x0000000F, 0, 0,
                3600 + int(seconds), 17)))
            frames.append(frame(LOG_GPS1_POS, struct.pack(
                '<III3df3fBHH', ts, GNSS_RTK_INT, 0, 46.2, 6.1, 400.0, 48.0,
                0.012, 0.011, 0.02, 21, 7, 120)))
            frames.append(frame(LOG_GPS1_HDT, struct.pack(
                '<IHI5fBB', ts, 1 << 6, 0, 123.5, 0.2, 1.0, 0.4, 1.25, 16, 14)))
    with open(path, 'wb') as handle:
        handle.write(b''.join(frames))


@pytest.mark.launch_test
def generate_test_description():
    fd, replay_path = tempfile.mkstemp(suffix='.bin', prefix='sbg_health_')
    os.close(fd)
    write_capture(replay_path)

    def driver(namespace, parameters, **kwargs):
        return launch_ros.actions.LifecycleNode(
            package='sbg_driver', executable='sbg_driver_node', name='sbg_driver',
            namespace=namespace, output='screen', parameters=[parameters], **kwargs)

    # Identity is configured, and deliberately unlike the ROS namespace.
    front = driver('front', {
        'transport.type': 'file',
        'transport.file.path': replay_path,
        'transport.file.real_time_pace': True,
        'health.vehicle_id': 'uav6',
        'health.instance': 'ins',
        'health.publish_interval_s': 0.2,
        'health.stale_after_s': 2.0,
        'health.stream_timeout_s': 1.0,
        'health.required_gnss_fix': 'rtk_fixed',
    })
    # No identity configured: the fallbacks (VEHICLE_ID, the namespace) apply.
    broken = driver('broken', {
        'transport.type': 'file',
        'transport.file.path': replay_path + '.missing',
        'health.publish_interval_s': 0.2,
    }, additional_env={'VEHICLE_ID': 'uav9'})

    # Stopped by signal while active and streaming / while only configured.
    stopper = driver('stopper', {
        'transport.type': 'file',
        'transport.file.path': replay_path,
        'transport.file.real_time_pace': True,
        'health.vehicle_id': 'uav6',
        'health.publish_interval_s': 0.2,
        'health.stale_after_s': 2.0,
    })
    terminator = driver('terminator', {
        'transport.type': 'file',
        'transport.file.path': replay_path,
        'health.vehicle_id': 'uav6',
        'health.publish_interval_s': 0.2,
        'health.stale_after_s': 2.0,
    })

    return (
        launch.LaunchDescription(
            [front, broken, stopper, terminator, launch_testing.actions.ReadyToTest()]),
        {'front': front, 'broken': broken, 'stopper': stopper, 'terminator': terminator},
    )


def values(status):
    return {kv.key: kv.value for kv in status.values}


class TestHealthDiagnostics(unittest.TestCase):

    @classmethod
    def setUpClass(cls):
        rclpy.init()
        cls.node = rclpy.create_node('health_test_client')
        cls.arrays = []
        cls.headings = []
        cls.node.create_subscription(
            DiagnosticArray, '/diagnostics', cls.arrays.append,
            QoSProfile(depth=200, reliability=ReliabilityPolicy.RELIABLE,
                       durability=DurabilityPolicy.VOLATILE))
        cls.node.create_subscription(
            Imu, '/front/gps/heading', cls.headings.append, qos_profile_sensor_data)
        cls.transitions = {'stopper': [], 'terminator': []}
        for name, seen in cls.transitions.items():
            cls.node.create_subscription(
                lifecycle_msgs.msg.TransitionEvent, f'/{name}/sbg_driver/transition_event',
                lambda msg, seen=seen: seen.append(msg.goal_state.label), 10)
        cls.clients = {}
        for name in ('front', 'broken', 'stopper', 'terminator'):
            client = cls.node.create_client(
                lifecycle_msgs.srv.ChangeState, f'/{name}/sbg_driver/change_state')
            assert client.wait_for_service(timeout_sec=20.0), f'{name}: no change_state service'
            cls.clients[name] = client

    @classmethod
    def tearDownClass(cls):
        cls.node.destroy_node()
        rclpy.shutdown()

    # ---- helpers -----------------------------------------------------------

    def change_state(self, name, transition_id):
        request = lifecycle_msgs.srv.ChangeState.Request()
        request.transition.id = transition_id
        future = self.clients[name].call_async(request)
        rclpy.spin_until_future_complete(self.node, future, timeout_sec=15.0)
        self.assertTrue(future.done(), f'{name}: transition {transition_id} did not return')
        return future.result().success

    def spin(self, seconds):
        deadline = time.monotonic() + seconds
        while time.monotonic() < deadline:
            rclpy.spin_once(self.node, timeout_sec=0.05)

    def statuses(self, instance, start=0):
        """Every (array index, status) reported for a fleet instance, in arrival order."""
        found = []
        for index, array in enumerate(self.arrays[start:], start):
            for status in array.status:
                if values(status).get('health.instance') == instance:
                    found.append((index, status))
        return found

    def component(self, instance, name, start=0):
        return [s for _, s in self.statuses(instance, start) if s.name == f'{instance}: {name}']

    def wait_for(self, predicate, timeout=10.0, what='condition'):
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            rclpy.spin_once(self.node, timeout_sec=0.05)
            result = predicate()
            if result:
                return result
        self.fail(f'timed out waiting for {what}')

    def stop_by_signal(self, name, action, proc_info, proc_output, signals):
        """Signal a driver process and check it left through its lifecycle shutdown."""
        start = len(self.arrays)
        pid = action.process_details['pid']
        began = time.monotonic()
        for signum in signals:
            os.kill(pid, signum)
        proc_info.assertWaitForShutdown(process=action, timeout=10)
        elapsed = time.monotonic() - began
        self.assertLess(elapsed, 5.0, f'{name} took {elapsed:.1f} s to stop')

        # The last thing on the wire is the node saying it shut down, and a
        # lifecycle manager sees it reach finalized.
        self.spin(1.0)
        reports = self.component(name, 'service', start)
        self.assertTrue(reports, f'{name}: no health report after the signal')
        self.assertEqual(values(reports[-1])['state'], 'shutdown')
        self.assertEqual(reports[-1].level, DiagnosticStatus.OK)
        self.assertEqual([values(s)['state'] for s in reports].count('shutdown'), 1)
        stream = self.component(name, 'stream', start)[-1]
        self.assertEqual(values(stream)['health.availability'], 'paused')
        self.assertEqual(self.transitions[name][-2:], ['shuttingdown', 'finalized'])

        output = ''.join(event.text.decode() for event in proc_output[action])
        self.assertIn('shutting down', output)
        # rclcpp_lifecycle's complaint about a node destroyed outside finalized.
        self.assertNotIn('is not shut down', output)
        count = len(self.statuses(name))
        self.spin(1.0)
        self.assertEqual(len(self.statuses(name)), count, f'{name} still publishing')

    # ---- tests, in order ---------------------------------------------------

    def test_01_unconfigured_node_reports_nothing_then_standby_is_healthy(self):
        self.spin(1.0)
        self.assertEqual(self.statuses('ins'), [], 'health before configure')

        self.assertTrue(self.change_state(
            'front', lifecycle_msgs.msg.Transition.TRANSITION_CONFIGURE))
        service = self.wait_for(
            lambda: self.component('ins', 'service'), what='the first standby report')[-1]
        # ROS `byte`: compare against the generated constants, never bare ints.
        self.assertEqual(service.level, DiagnosticStatus.OK)
        self.assertEqual(service.message, 'standby')
        self.assertEqual(service.hardware_id, '')
        fields = values(service)
        self.assertEqual(fields['state'], 'standby')
        self.assertEqual(fields['transport'], 'file')
        self.assertEqual(fields['health.vehicle_id'], 'uav6')
        self.assertEqual(fields['health.service'], 'sbg')
        self.assertEqual(fields['health.publish_interval_s'], '0.2')
        self.assertEqual(fields['health.stale_after_s'], '2')
        self.assertEqual(fields['health.availability'], 'current')

        # The complete inventory, with nothing demanded of the device yet.
        names = {s.name for _, s in self.statuses('ins')}
        self.assertEqual(names, {
            'ins: service', 'ins: stream', 'ins: device', 'ins: sensor alerts',
            'ins: temperature', 'ins: ekf', 'ins: imu'})
        for name in ('stream', 'device', 'temperature', 'ekf'):
            status = self.component('ins', name)[-1]
            self.assertEqual(values(status)['health.availability'], 'paused', name)
            self.assertNotIn('health.error', values(status), name)
        self.assertEqual(self.component('ins', 'stream')[-1].level, DiagnosticStatus.OK)
        self.assertEqual(self.component('ins', 'device')[-1].level, DiagnosticStatus.STALE)

    def test_02_publisher_is_reliable_volatile_with_system_clock_stamps(self):
        infos = [i for i in self.node.get_publishers_info_by_topic('/diagnostics')
                 if i.node_namespace == '/front']
        self.assertEqual(len(infos), 1)
        self.assertEqual(infos[0].topic_type, 'diagnostic_msgs/msg/DiagnosticArray')
        self.assertEqual(infos[0].qos_profile.reliability, ReliabilityPolicy.RELIABLE)
        self.assertEqual(infos[0].qos_profile.durability, DurabilityPolicy.VOLATILE)
        stamp = self.arrays[-1].header.stamp
        self.assertLess(abs(time.time() - (stamp.sec + stamp.nanosec * 1e-9)), 30.0)

    def test_03_active_reports_measurements_fix_grade_and_a_fault_that_clears(self):
        start = len(self.arrays)
        self.assertTrue(self.change_state(
            'front', lifecycle_msgs.msg.Transition.TRANSITION_ACTIVATE))

        def temperature():
            return [s for s in self.component('ins', 'temperature', start)
                    if values(s).get('health.availability') == 'current']
        temp = self.wait_for(temperature, what='a current temperature')[-1]
        self.assertEqual(temp.level, DiagnosticStatus.OK)
        self.assertEqual(values(temp)['temp.imu_c'], '37.5')

        def cleared():
            return [s for s in self.component('ins', 'alert/CPU', start)
                    if values(s)['alert.active'] == 'false']
        self.wait_for(cleared, what='the CPU alert to clear')
        alerts = self.component('ins', 'alert/CPU', start)
        raised = [s for s in alerts if values(s)['alert.active'] == 'true']
        self.assertTrue(raised, 'the CPU headroom fault was never reported')
        self.assertEqual(raised[0].level, DiagnosticStatus.WARN)
        self.assertEqual(raised[0].message, 'insufficient CPU headroom')
        self.assertEqual(values(raised[0])['alert.code'], 'CPU')
        self.assertEqual(values(raised[0])['alert.flag'], 'SBG_ECOM_GENERAL_CPU_OK')
        self.assertEqual(cleared()[-1].level, DiagnosticStatus.OK)
        self.assertEqual(cleared()[-1].message, 'cleared')
        summary = self.component('ins', 'sensor alerts', start)
        self.assertIn('1', {values(s).get('alerts.active') for s in summary})
        self.assertEqual(values(summary[-1])['alerts.active'], '0')

        self.assertEqual(values(self.component('ins', 'service', start)[-1])['state'], 'active')
        stream = self.component('ins', 'stream', start)[-1]
        self.assertEqual(stream.level, DiagnosticStatus.OK)
        self.assertGreater(int(values(stream)['logs.received']), 0)
        device = values(self.component('ins', 'device', start)[-1])
        self.assertEqual(device['device.general_status'], '0x007f')
        self.assertEqual(device['cpu.usage_pct'], '17')
        self.assertGreaterEqual(int(device['uptime_s']), 3600)
        self.assertEqual(device['health.metric.supply.power_w.state'], 'unsupported')
        ekf = self.component('ins', 'ekf', start)[-1]
        self.assertEqual(ekf.level, DiagnosticStatus.OK)
        self.assertEqual(values(ekf)['ekf.solution_mode'], 'NAV_POSITION')
        gnss = self.component('ins', 'gnss', start)[-1]
        self.assertEqual(gnss.level, DiagnosticStatus.OK)  # meets the required rtk_fixed
        self.assertEqual(gnss.message, 'RTK fixed')
        self.assertEqual(values(gnss)['gnss.rtk'], 'fixed')
        self.assertEqual(values(gnss)['gnss.sv_used'], '21')
        heading = self.component('ins', 'gnss heading', start)[-1]
        self.assertEqual(heading.level, DiagnosticStatus.OK)
        self.assertEqual(values(heading)['gnss.heading_deg'], '123.5')

        # No electrical measurement is ever invented for this device.
        for _, status in self.statuses('ins', start):
            for key in ('supply.voltage_v', 'supply.current_a', 'supply.power_w'):
                self.assertNotIn(key, values(status), status.name)

    def test_04_gnss_heading_is_published_as_an_orientation_only_imu(self):
        self.wait_for(lambda: self.headings, what='a /front/gps/heading message')
        msg = self.headings[-1]
        self.assertEqual(msg.header.frame_id, 'gps_link')
        yaw = math.degrees(2.0 * math.atan2(msg.orientation.z, msg.orientation.w))
        self.assertAlmostEqual(yaw, 123.5, places=3)  # NED default: clockwise from north
        self.assertAlmostEqual(msg.orientation_covariance[8], math.radians(0.2) ** 2, places=9)
        self.assertEqual(msg.angular_velocity_covariance[0], -1.0)
        self.assertEqual(msg.linear_acceleration_covariance[0], -1.0)

    def test_05_end_of_stream_is_an_error_and_cached_values_go_stale_not_fresh(self):
        start = len(self.arrays)

        def stale_temperature():
            return [s for s in self.component('ins', 'temperature', start)
                    if s.level == DiagnosticStatus.STALE]
        last = self.wait_for(
            stale_temperature, timeout=CAPTURE_SECONDS + 8.0, what='a stale temperature')[-1]
        fields = values(last)
        self.assertEqual(fields['temp.imu_c'], '37.5')       # last known, kept
        self.assertEqual(fields['health.last_level'], '0')
        self.assertEqual(fields['health.availability'], 'stale')
        self.assertIn('IMU log', fields['health.error'])
        self.assertGreater(float(fields['health.sample_age_s']), 2.0)

        stream = self.component('ins', 'stream', start)[-1]
        self.assertEqual(stream.level, DiagnosticStatus.ERROR)
        self.assertIn('no logs for', stream.message)
        self.assertEqual(values(stream)['health.availability'], 'current')

        # Between "current" and "stale" the reading was marked as retrying,
        # and through all of it the heartbeat never renewed the sample.
        history = self.component('ins', 'temperature', start)
        self.assertIn('retrying', [values(s)['health.availability'] for s in history])
        silent = [values(s) for s in history if values(s)['health.availability'] != 'current']
        self.assertEqual(len({v['health.sample_id'] for v in silent}), 1)
        ages = [float(v['health.sample_age_s']) for v in silent]
        self.assertEqual(ages, sorted(ages))
        self.assertGreater(ages[-1], ages[0])

    def test_06_one_publisher_with_a_strictly_increasing_sequence(self):
        seen = {}
        for index, status in self.statuses('ins'):
            fields = values(status)
            seen.setdefault(fields['health.publisher_id'], {})[index] = \
                int(fields['health.sequence'])
            self.assertTrue(status.name.startswith('ins: '), status.name)
            self.assertEqual(fields['health.vehicle_id'], 'uav6')
        self.assertEqual(len(seen), 1, 'more than one publisher id for one process')
        sequences = list(next(iter(seen.values())).values())
        self.assertGreater(len(sequences), 10)
        self.assertEqual(sequences, sorted(set(sequences)))

    def test_07_deactivation_is_a_confirmed_standby_with_last_known_values(self):
        start = len(self.arrays)
        self.assertTrue(self.change_state(
            'front', lifecycle_msgs.msg.Transition.TRANSITION_DEACTIVATE))

        def standby():
            return [s for s in self.component('ins', 'service', start)
                    if values(s)['state'] == 'standby']
        self.assertEqual(self.wait_for(standby, what='standby')[-1].level, DiagnosticStatus.OK)
        self.spin(0.5)
        stream = self.component('ins', 'stream', start)[-1]
        self.assertEqual(stream.level, DiagnosticStatus.OK)  # silence is expected now
        temp = self.component('ins', 'temperature', start)[-1]
        self.assertEqual(values(temp)['health.availability'], 'paused')
        self.assertEqual(values(temp)['temp.imu_c'], '37.5')
        self.assertEqual(temp.level, DiagnosticStatus.OK)

    def test_08_failed_activation_is_reported_as_an_error(self):
        self.assertTrue(self.change_state(
            'broken', lifecycle_msgs.msg.Transition.TRANSITION_CONFIGURE))
        # Fallback identity: the namespace as instance, VEHICLE_ID as vehicle.
        service = self.wait_for(
            lambda: self.component('broken', 'service'), what="broken's standby report")[-1]
        self.assertEqual(service.level, DiagnosticStatus.OK)
        self.assertEqual(values(service)['health.vehicle_id'], 'uav9')

        start = len(self.arrays)
        self.assertFalse(self.change_state(
            'broken', lifecycle_msgs.msg.Transition.TRANSITION_ACTIVATE),
            'activating against a missing capture should be rejected')

        def failed():
            return [s for s in self.component('broken', 'service', start)
                    if s.level == DiagnosticStatus.ERROR]
        service = self.wait_for(failed, what='the activation failure')[-1]
        self.assertTrue(service.message.startswith('activation failed: device open failed'),
                        service.message)
        self.assertEqual(values(service)['state'], 'standby')
        # A failed attempt is not confused with the healthy instance next to it.
        self.assertEqual(self.component('ins', 'service')[-1].level, DiagnosticStatus.OK)

    def test_09_shutdown_leaves_the_final_state_and_then_stops(self):
        start = len(self.arrays)
        self.assertTrue(self.change_state(
            'front', lifecycle_msgs.msg.Transition.TRANSITION_INACTIVE_SHUTDOWN))

        def shutdown():
            return [s for s in self.component('ins', 'service', start)
                    if values(s)['state'] == 'shutdown']
        self.wait_for(shutdown, what='the final shutdown report')
        self.spin(0.5)
        count = len(self.statuses('ins'))
        self.spin(1.5)  # seven heartbeat intervals
        self.assertEqual(len(self.statuses('ins')), count, 'still publishing after shutdown')
        self.assertEqual(values(self.component('ins', 'service')[-1])['state'], 'shutdown')

    def test_10_sigint_while_streaming_runs_the_lifecycle_shutdown(
            self, proc_info, proc_output, stopper):
        self.assertTrue(self.change_state(
            'stopper', lifecycle_msgs.msg.Transition.TRANSITION_CONFIGURE))
        self.assertTrue(self.change_state(
            'stopper', lifecycle_msgs.msg.Transition.TRANSITION_ACTIVATE))

        def streaming():
            return [s for s in self.component('stopper', 'stream')
                    if int(values(s)['logs.received']) > 0]
        self.wait_for(streaming, what='stopper to stream')
        # Twice, back to back: Ctrl-C under `ros2 launch` reaches the node once
        # from the terminal and once from launch, and the second must not cut
        # the shutdown short.
        self.stop_by_signal(
            'stopper', stopper, proc_info, proc_output, [signal.SIGINT, signal.SIGINT])

    def test_11_sigterm_while_inactive_runs_the_lifecycle_shutdown(
            self, proc_info, proc_output, terminator):
        self.assertTrue(self.change_state(
            'terminator', lifecycle_msgs.msg.Transition.TRANSITION_CONFIGURE))
        self.wait_for(lambda: self.component('terminator', 'service'), what='terminator standby')
        self.stop_by_signal(
            'terminator', terminator, proc_info, proc_output, [signal.SIGTERM])


@launch_testing.post_shutdown_test()
class TestProcessExit(unittest.TestCase):

    def test_exit_codes_ok(self, proc_info, front, broken, stopper, terminator):
        # Every process ends on a signal (two in the tests above, the others
        # when launch tears down) and each must still exit 0.
        for process in (front, broken, stopper, terminator):
            launch_testing.asserts.assertExitCodes(proc_info, process=process)

    def test_no_node_was_destroyed_outside_finalized(self, proc_output):
        output = ''.join(event.text.decode() for event in proc_output)
        self.assertNotIn('is not shut down', output)
