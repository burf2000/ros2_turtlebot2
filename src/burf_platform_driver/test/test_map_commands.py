"""Unit tests for the BurfPlatform driver's map-management command handlers.

These exercise the command handlers (cmd_map_save, cmd_map_load, cmd_set_pose)
with the ROS2 service clients / publishers mocked, so they run with no ROS2
install and no robot. We stub the heavy ROS2 modules in sys.modules before
importing the driver, then build a BurfPlatformDriver instance via
object.__new__ (skipping the real __init__, which would spin up ROS2 + a
WebSocket thread) and inject the mocks the handlers depend on.

Run:  python -m pytest test/   (or)  python -m unittest discover -s test
"""

import base64
import json
import os
import sys
import tempfile
import types
import unittest
from unittest import mock


# ─── Stub the ROS2 modules the driver imports at module scope ────────────────
def _install_ros2_stubs():
    def _mod(name):
        m = types.ModuleType(name)
        sys.modules[name] = m
        return m

    rclpy = _mod('rclpy')
    rclpy.init = lambda *a, **k: None
    rclpy.shutdown = lambda *a, **k: None
    rclpy.spin = lambda *a, **k: None

    duration_mod = _mod('rclpy.duration')

    class _Duration:
        def __init__(self, *a, **k):
            pass
    duration_mod.Duration = _Duration
    rclpy.duration = duration_mod

    time_mod = _mod('rclpy.time')

    class _Time:
        def __init__(self, *a, **k):
            pass
    time_mod.Time = _Time
    rclpy.time = time_mod

    node_mod = _mod('rclpy.node')
    node_mod.Node = type('Node', (), {})

    qos_mod = _mod('rclpy.qos')
    qos_mod.QoSProfile = type('QoSProfile', (), {})
    qos_mod.QoSDurabilityPolicy = type('QoSDurabilityPolicy', (), {'TRANSIENT_LOCAL': 1})
    qos_mod.QoSReliabilityPolicy = type('QoSReliabilityPolicy', (), {'RELIABLE': 1})

    cbg_mod = _mod('rclpy.callback_groups')
    cbg_mod.ReentrantCallbackGroup = type('ReentrantCallbackGroup', (), {})
    cbg_mod.MutuallyExclusiveCallbackGroup = type('MutuallyExclusiveCallbackGroup', (), {})

    exec_mod = _mod('rclpy.executors')
    exec_mod.MultiThreadedExecutor = type('MultiThreadedExecutor', (), {})

    # Message packages — only the names referenced at import time matter.
    sensor_msgs = _mod('sensor_msgs')
    sensor_msgs_msg = _mod('sensor_msgs.msg')
    for n in ('Image', 'Imu', 'CompressedImage', 'LaserScan', 'BatteryState'):
        setattr(sensor_msgs_msg, n, type(n, (), {}))
    sensor_msgs.msg = sensor_msgs_msg

    nav_msgs = _mod('nav_msgs')
    nav_msgs_msg = _mod('nav_msgs.msg')
    for n in ('Odometry', 'OccupancyGrid'):
        setattr(nav_msgs_msg, n, type(n, (), {}))
    nav_msgs.msg = nav_msgs_msg

    geometry_msgs = _mod('geometry_msgs')
    geometry_msgs_msg = _mod('geometry_msgs.msg')

    class _Vec:
        def __init__(self):
            self.x = 0.0
            self.y = 0.0
            self.z = 0.0
            self.w = 1.0

    class _PoseInner:
        def __init__(self):
            self.position = _Vec()
            self.orientation = _Vec()

    class _PoseWrap:
        def __init__(self):
            self.pose = _PoseInner()
            self.covariance = [0.0] * 36

    class _Header:
        def __init__(self):
            self.stamp = None
            self.frame_id = ''

    class PoseWithCovarianceStamped:
        def __init__(self):
            self.header = _Header()
            self.pose = _PoseWrap()

    geometry_msgs_msg.Twist = type('Twist', (), {})
    geometry_msgs_msg.PoseWithCovarianceStamped = PoseWithCovarianceStamped
    geometry_msgs.msg = geometry_msgs_msg

    tf2 = _mod('tf2_ros')
    tf2.Buffer = type('Buffer', (), {})
    tf2.TransformListener = type('TransformListener', (), {})

    # Third-party deps the driver guards with try/except — provide nothing so
    # the ImportError paths are taken (PIL/cv2/aiortc/websockets/psutil/numpy).
    # numpy IS used in map_callback; provide the real one if available.
    for n in ('psutil', 'websockets', 'cv2', 'aiortc', 'aiortc.sdp', 'av', 'PIL'):
        if n not in sys.modules:
            sys.modules[n] = types.ModuleType(n)


_install_ros2_stubs()

# Make the package importable.
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))

from burf_platform_driver import burf_driver  # noqa: E402
from burf_platform_driver.path_planner import GridInfo  # noqa: E402


# ─── Test doubles ────────────────────────────────────────────────────────────
class _FakeFuture:
    """Future whose add_done_callback fires synchronously with `result`."""
    def __init__(self, result=None, exc=None):
        self._result = result
        self._exc = exc

    def result(self):
        if self._exc is not None:
            raise self._exc
        return self._result

    def add_done_callback(self, cb):
        cb(self)


class _FakeServiceClient:
    """Mimics an rclpy service client for the handlers under test."""
    def __init__(self, ready=True, exc=None):
        self._ready = ready
        self._exc = exc
        self.calls = []          # captured request objects
        self.RequestType = None  # set by tests via the *_PoseGraph attribute

    def service_is_ready(self):
        return self._ready

    def wait_for_service(self, timeout_sec=0.0):
        return self._ready

    def call_async(self, request):
        self.calls.append(request)
        return _FakeFuture(result=object(), exc=self._exc)


class _Req:
    """Generic request object with a settable filename / match_type / pose."""
    def __init__(self):
        self.filename = None
        self.match_type = None

        class _P:
            def __init__(self):
                self.x = 0.0
                self.y = 0.0
                self.theta = 0.0
        self.initial_pose = _P()


def _make_request_type():
    rt = mock.Mock()
    rt.Request = _Req
    return rt


class _FakeLogger:
    def info(self, *a, **k):
        pass

    def warning(self, *a, **k):
        pass

    def error(self, *a, **k):
        pass

    def debug(self, *a, **k):
        pass


class _FakeClock:
    class _Now:
        def to_msg(self):
            return None

    def now(self):
        return self._Now()


def _new_driver():
    """Build a driver instance without running __init__."""
    d = burf_driver.BurfPlatformDriver.__new__(burf_driver.BurfPlatformDriver)
    d.robot_id = 'test-robot'
    d.map_frame = 'map'
    d.ws_loop = None  # so _emit_* short-circuits the asyncio send
    d._emitted = []   # capture emitted result dicts
    d.get_logger = lambda: _FakeLogger()
    d.get_clock = lambda: _FakeClock()
    d.get_timestamp = lambda: '2026-01-01T00:00:00.000Z'
    return d


class MapSaveTests(unittest.TestCase):
    def setUp(self):
        self.d = _new_driver()
        self.tmp = tempfile.mkdtemp()
        self.d.map_storage_dir = self.tmp
        self.d.enable_map_relay = True
        self.d.latest_map_info = GridInfo(
            width=100, height=120, resolution=0.05,
            origin_x=-1.0, origin_y=-2.0)
        cli = _FakeServiceClient(ready=True)
        self.d._serialize_cli = cli
        self.d._SerializePoseGraph = _make_request_type()
        self.cli = cli
        # capture emitted save results
        emitted = []
        self.emitted = emitted

        def _emit(name, ok, path=None, error=None, upload_blob=False):
            entry = {'name': name, 'ok': ok, 'path': path,
                     'error': error, 'upload_blob': upload_blob}
            # exercise the real blob read path on success
            if ok and upload_blob and path:
                blob = self.d._read_map_blob(path)
                if blob:
                    entry['posegraph_b64'] = blob['posegraph_b64']
                    entry['data_b64'] = blob['data_b64']
            emitted.append(entry)
        self.d._emit_map_save_result = _emit

    def test_serialize_called_and_blob_uploaded(self):
        name = 'ground-floor'
        # Pre-create the files slam_toolbox would write so the blob read works.
        with open(os.path.join(self.tmp, name + '.posegraph'), 'wb') as f:
            f.write(b'POSEGRAPH-BYTES')
        with open(os.path.join(self.tmp, name + '.data'), 'wb') as f:
            f.write(b'DATA-BYTES')

        self.d.handle_map_save_command({'name': name})

        # serialize_map service was called with the right (extensionless) path.
        self.assertEqual(len(self.cli.calls), 1)
        self.assertEqual(self.cli.calls[0].filename,
                         os.path.join(self.tmp, name))
        # emitted ok + the blob was read off disk and base64-encoded.
        self.assertEqual(len(self.emitted), 1)
        res = self.emitted[0]
        self.assertTrue(res['ok'])
        self.assertTrue(res['upload_blob'])
        self.assertEqual(
            base64.b64decode(res['posegraph_b64']), b'POSEGRAPH-BYTES')
        self.assertEqual(base64.b64decode(res['data_b64']), b'DATA-BYTES')

    def test_rejects_empty_name(self):
        self.d.handle_map_save_command({'name': '   '})
        self.assertEqual(len(self.cli.calls), 0)
        self.assertFalse(self.emitted[0]['ok'])

    def test_rejects_when_no_map_yet(self):
        self.d.latest_map_info = None
        self.d.handle_map_save_command({'name': 'x'})
        self.assertEqual(len(self.cli.calls), 0)
        self.assertFalse(self.emitted[0]['ok'])


class MapLoadTests(unittest.TestCase):
    def setUp(self):
        self.d = _new_driver()
        self.tmp = tempfile.mkdtemp()
        self.d.map_storage_dir = self.tmp
        self.d.enable_map_relay = True
        self.d._loaded_map_path = None
        cli = _FakeServiceClient(ready=True)
        self.d._deserialize_cli = cli
        self.d._DeserializePoseGraph = _make_request_type()
        self.cli = cli
        self.emitted = []
        self.d._emit_map_load_result = lambda name, ok, path=None, error=None: \
            self.emitted.append({'name': name, 'ok': ok, 'path': path,
                                 'error': error})

    def test_deserialize_called_with_written_file(self):
        pg = base64.b64encode(b'PG').decode()
        dt = base64.b64encode(b'DT').decode()
        self.d.handle_map_load_command(
            {'name': 'lab', 'posegraph_b64': pg, 'data_b64': dt})

        # The blob was written to disk.
        base = os.path.join(self.tmp, 'lab')
        with open(base + '.posegraph', 'rb') as f:
            self.assertEqual(f.read(), b'PG')
        with open(base + '.data', 'rb') as f:
            self.assertEqual(f.read(), b'DT')
        # deserialize_map called with that basename + LOCALIZE_AT_POSE.
        self.assertEqual(len(self.cli.calls), 1)
        self.assertEqual(self.cli.calls[0].filename, base)
        self.assertEqual(self.cli.calls[0].match_type, 3)
        # success emitted + remembered.
        self.assertTrue(self.emitted[0]['ok'])
        self.assertEqual(self.d._loaded_map_path, base)

    def test_missing_blob_rejected(self):
        self.d.handle_map_load_command({'name': 'lab'})
        self.assertEqual(len(self.cli.calls), 0)
        self.assertFalse(self.emitted[0]['ok'])

    def test_no_deserialize_client_rejected(self):
        self.d._deserialize_cli = None
        pg = base64.b64encode(b'PG').decode()
        dt = base64.b64encode(b'DT').decode()
        self.d.handle_map_load_command(
            {'name': 'lab', 'posegraph_b64': pg, 'data_b64': dt})
        self.assertFalse(self.emitted[0]['ok'])


class SetPoseTests(unittest.TestCase):
    def setUp(self):
        self.d = _new_driver()
        self.published = []
        pub = mock.Mock()
        pub.publish = lambda msg: self.published.append(msg)
        self.d.initialpose_pub = pub
        self.emitted = []
        self.d._emit_set_pose_result = \
            lambda ok, x=None, y=None, theta=None, error=None: \
            self.emitted.append({'ok': ok, 'x': x, 'y': y,
                                 'theta': theta, 'error': error})

    def test_publishes_pose_with_covariance_to_initialpose(self):
        self.d.handle_set_pose_command({'x': 1.5, 'y': -2.0, 'theta': 0.0})
        self.assertEqual(len(self.published), 1)
        msg = self.published[0]
        self.assertEqual(msg.header.frame_id, 'map')
        self.assertAlmostEqual(msg.pose.pose.position.x, 1.5)
        self.assertAlmostEqual(msg.pose.pose.position.y, -2.0)
        # theta=0 -> quaternion w=1, z=0
        self.assertAlmostEqual(msg.pose.pose.orientation.w, 1.0)
        self.assertAlmostEqual(msg.pose.pose.orientation.z, 0.0)
        # covariance seeded on the x/y/yaw diagonal.
        self.assertGreater(msg.pose.covariance[0], 0.0)
        self.assertGreater(msg.pose.covariance[7], 0.0)
        self.assertGreater(msg.pose.covariance[35], 0.0)
        self.assertTrue(self.emitted[0]['ok'])

    def test_bad_payload_rejected(self):
        self.d.handle_set_pose_command({'x': 'nope', 'y': 1, 'theta': 0})
        self.assertEqual(len(self.published), 0)
        self.assertFalse(self.emitted[0]['ok'])


# NOTE: HealthTests (compute_health self-diagnosis) removed — that feature was
# reverted out of the driver in 2c86f2c ("revert to known-good baseline"); the
# current driver has no health block, so its tests were stale.


if __name__ == '__main__':
    unittest.main()
