"""Validate RGB-D units, geometry, missing inputs and acceptance gates."""

from collections import Counter, deque
from types import SimpleNamespace
from unittest.mock import MagicMock

import numpy as np
import pytest
from sensor_msgs.msg import CameraInfo, Image
from tf2_ros import TransformException

from ackermann_line_following_controller import rgbd_audit as module


def camera_pair():
    rgb, depth, info = Image(), Image(), CameraInfo()
    for message in (rgb, depth, info):
        message.header.frame_id = 'camera_optical'
        message.width, message.height = 4, 3
        message.header.stamp.sec = 1
    depth.encoding = '32FC1'
    info.k = [4., 0., 2., 0., 4., 1., 0., 0., 1.]
    return rgb, depth, info


def test_depth_units_and_invalid_pixels():
    metres = np.array([[0., 1., 2., np.nan, np.inf]], dtype=np.float32)
    report = module.depth_statistics(metres, '32FC1')
    assert report['valid_fraction'] == pytest.approx(0.4)
    assert report['median_depth_m'] == pytest.approx(1.5)
    millimetres = np.array([[0, 1000, 2000, 65535]], dtype=np.uint16)
    assert module.depth_statistics(millimetres, '16UC1')['median_depth_m'] == pytest.approx(1.5)
    assert module.depth_statistics(np.zeros((1, 1), dtype=np.float32), '32FC1')['median_depth_m'] is None


@pytest.mark.parametrize('data,encoding,near,far', [
    (np.zeros((2, 2)), '32FC1', 0.2, 8),
    (np.zeros((2, 2), dtype=np.uint16), 'mono16', 0.2, 8),
    (np.zeros((0, 0), dtype=np.float32), '32FC1', 0.2, 8),
    (np.zeros((2, 2), dtype=np.float32), '32FC1', 8, 0.2),
    (np.zeros((2, 2), dtype=np.float32), '32FC1', float('nan'), 8),
])
def test_depth_rejects_invalid_inputs(data, encoding, near, far):
    with pytest.raises(ValueError):
        module.depth_statistics(data, encoding, near, far)


def test_camera_pair_and_stamp_delta():
    rgb, depth, info = camera_pair()
    depth.header.stamp.nanosec = 10_000_000
    assert module.validate_camera_pair(rgb, depth, info) == pytest.approx(0.01)
    info.k[0] = 0.
    with pytest.raises(ValueError, match='intrinsics'):
        module.validate_camera_pair(rgb, depth, info)


@pytest.mark.parametrize('kind', ['frame', 'dimensions', 'principal_point'])
def test_camera_rejects_unregistered_data(kind):
    rgb, depth, info = camera_pair()
    if kind == 'frame':
        depth.header.frame_id = 'different_camera'
    elif kind == 'dimensions':
        depth.width = 1
    else:
        info.k[2] = 40.
    with pytest.raises(ValueError):
        module.validate_camera_pair(rgb, depth, info)


def test_summary_requires_actual_data_and_all_gates():
    assert not module.audit_summary([], {}, {}, 1)['passed']
    sample = {'tf_valid': True, 'sync_delta_s': 0.01, 'valid_fraction': 0.8}
    health = {key: {'count': 1, 'nonmonotonic_stamps': 0, 'max_wall_gap_s': 0.1,
                    'last_receipt_age_s': 0.1} for key in ('rgb', 'depth', 'camera_info')}
    assert module.audit_summary([sample], {}, health, 1)['passed']
    sample['tf_valid'] = False
    assert not module.audit_summary([sample], {}, health, 1)['passed']
    sample['tf_valid'] = True
    assert not module.audit_summary([sample], {'invalid': 1}, health, 1)['passed']
    health['depth']['nonmonotonic_stamps'] = 1
    assert not module.audit_summary([sample], {}, health, 1)['passed']


def fake_audit():
    node = SimpleNamespace(bridge=MagicMock(), buffer=MagicMock(), health=MagicMock(),
                           samples=[], errors=Counter(), pending=deque(), camera_metadata=None)
    node.bridge.imgmsg_to_cv2.side_effect = lambda msg, desired_encoding: (
        np.zeros((3, 4, 3), dtype=np.uint8) if desired_encoding == 'rgb8'
        else np.ones((3, 4), dtype=np.float32))
    return node


def test_received_and_matched_callbacks():
    node = fake_audit()
    rgb, depth, info = camera_pair()
    module.RgbdAudit._received(node, 'rgb', rgb)
    node.health.observe.assert_called_once()
    node.bridge.imgmsg_to_cv2.return_value = np.ones((3, 4), dtype=np.float32)
    module.RgbdAudit._matched(node, rgb, depth, info)
    assert len(node.pending) == 1
    assert node.camera_metadata['width'] == 4
    info.k[0] = 0.
    module.RgbdAudit._matched(node, rgb, depth, info)
    assert sum(node.errors.values()) == 1


def test_tf_wait_is_bounded_and_failed_frames_are_counted(monkeypatch):
    node = fake_audit()
    header = camera_pair()[1].header
    node.pending.append((1., header, {}))
    node.buffer.lookup_transform.side_effect = TransformException('missing')
    monkeypatch.setattr(module.time, 'monotonic', lambda: 1.1)
    module.RgbdAudit._drain(node)
    assert len(node.pending) == 1
    monkeypatch.setattr(module.time, 'monotonic', lambda: 1.4)
    module.RgbdAudit._drain(node)
    assert not node.pending
    assert node.samples == [{'tf_valid': False}]


def test_tf_success_records_map_pose():
    node = fake_audit()
    node.pending.append((0., camera_pair()[1].header, {}))
    translation = node.buffer.lookup_transform.return_value.transform.translation
    translation.x, translation.y, translation.z = 1., 2., 3.
    module.RgbdAudit._drain(node)
    assert node.samples[0]['tf_valid']
    assert node.samples[0]['map_position'] == [1., 2., 3.]


def test_constructor_uses_bounded_sensor_synchronization(monkeypatch):
    monkeypatch.setattr(module.Node, '__init__', lambda *args, **kwargs: None)
    timer = MagicMock()
    monkeypatch.setattr(module.Node, 'create_timer', timer)
    monkeypatch.setattr(module, 'Buffer', MagicMock())
    monkeypatch.setattr(module, 'TransformListener', MagicMock())
    monkeypatch.setattr(module.message_filters, 'Subscriber', MagicMock())
    sync = MagicMock()
    monkeypatch.setattr(module.message_filters, 'ApproximateTimeSynchronizer', sync)
    node = module.RgbdAudit(True)
    assert len(node.inputs) == 3
    assert sync.call_args.kwargs == {'queue_size': 15, 'slop': 0.02}
    timer.assert_called_once()


def test_tf_queue_overflow_is_reported():
    node = fake_audit()
    node.bridge.imgmsg_to_cv2.return_value = np.ones((3, 4), dtype=np.float32)
    for _ in range(16):
        module.RgbdAudit._matched(node, *camera_pair())
    assert len(node.pending) == 15
    assert node.errors['tf_queue_overflow'] == 1


def test_cli_missing_data_fails_and_cleans_up(tmp_path, monkeypatch):
    node = fake_audit()
    node.health.report.return_value = {}
    node.destroy_node = MagicMock()
    monkeypatch.setattr(module, 'RgbdAudit', lambda *args: node)
    monkeypatch.setattr(module.rclpy, 'init', MagicMock())
    monkeypatch.setattr(module.rclpy, 'shutdown', MagicMock())
    monkeypatch.setattr(module.rclpy, 'ok', lambda: False)
    path = tmp_path / 'result.json'
    with pytest.raises(SystemExit):
        module.main(['--output', str(path)])
    assert path.is_file()
    node.destroy_node.assert_called_once()


def test_cli_rejects_invalid_duration():
    with pytest.raises(SystemExit):
        module.main(['--output', 'unused', '--seconds', 'nan'])
