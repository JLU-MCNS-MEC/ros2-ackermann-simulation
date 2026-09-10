"""Test visual navigation preprocessing, training and shadow metrics."""

import json
import math

import cv2
import numpy as np
import pytest

from ackermann_line_following_controller.visual_policy import (
    OpenCvVisualPolicy,
    ShadowMetrics,
    decode_policy_output,
    draw_policy_overlay,
    draw_status_overlay,
    encode_policy_input,
    encode_policy_target,
    normalize_angle,
    quaternion_to_yaw,
    relative_goal,
)
from ackermann_line_following_controller.visual_policy_train import (
    load_training_data,
    train_policy,
    validation_indices,
)
from ackermann_line_following_controller import visual_policy_train


def test_relative_goal_and_angle_wrap():
    distance, bearing = relative_goal(1.0, 2.0, math.pi / 2, 1.0, 4.0)
    assert distance == pytest.approx(2.0)
    assert bearing == pytest.approx(0.0)
    assert normalize_angle(3.0 * math.pi) == pytest.approx(math.pi)


@pytest.mark.parametrize(
    'function,args',
    [
        (normalize_angle, (float('nan'),)),
        (relative_goal, (0.0, 0.0, 0.0, float('inf'), 0.0)),
        (quaternion_to_yaw, (0.0, 0.0, 0.0, 0.0)),
    ],
)
def test_pose_helpers_reject_invalid_values(function, args):
    with pytest.raises(ValueError):
        function(*args)


def test_quaternion_to_yaw_normalizes_input():
    assert quaternion_to_yaw(0.0, 0.0, math.sqrt(2.0), math.sqrt(2.0)) == (
        pytest.approx(math.pi / 2)
    )


def test_policy_encoding_and_bounded_decoding():
    image = np.full((8, 12, 3), 127, dtype=np.uint8)
    row = encode_policy_input(image, 15.0, math.pi / 2, width=4, height=3)
    assert row.shape == (1, 4 * 3 * 3 + 3)
    assert row[0, -3:].tolist() == pytest.approx([1.0, 1.0, 0.0])
    target = encode_policy_target(0.7, -1.6)
    assert target.tolist() == pytest.approx([1.0, -1.0])
    assert decode_policy_output(np.asarray([2.0, -2.0])) == pytest.approx(
        (0.35, -0.8)
    )


def test_policy_encoding_rejects_bad_image_and_prediction():
    with pytest.raises(ValueError, match='BGR'):
        encode_policy_input(np.empty((0, 0), dtype=np.uint8), 1.0, 0.0)
    with pytest.raises(ValueError, match='two finite'):
        decode_policy_output(np.asarray([0.1, float('nan')]))
    image = np.zeros((4, 4, 3), dtype=np.uint8)
    with pytest.raises(ValueError, match='dimensions'):
        encode_policy_input(image, 1.0, 0.0, width=0)
    with pytest.raises(ValueError, match='max_goal_distance'):
        encode_policy_input(image, 1.0, 0.0, max_goal_distance=0.0)
    with pytest.raises(ValueError, match='nonnegative'):
        encode_policy_input(image, -1.0, 0.0)
    with pytest.raises(ValueError, match='speed limits'):
        encode_policy_target(0.1, 0.1, max_linear_speed=0.0)
    with pytest.raises(ValueError, match='speed limits'):
        decode_policy_output(np.zeros(2), max_angular_speed=float('nan'))


def test_overlay_preserves_shape_and_draws_pixels():
    image = np.zeros((120, 200, 3), dtype=np.uint8)
    overlay = draw_policy_overlay(
        image, 2.0, 0.2, (0.2, 0.1), (0.18, 0.08)
    )
    assert overlay.shape == image.shape
    assert np.any(overlay != image)
    status = draw_status_overlay(image, 'WAITING FOR GOAL')
    assert status.shape == image.shape
    assert np.any(status != image)
    with pytest.raises(ValueError, match='status'):
        draw_status_overlay(image, ' ')
    with pytest.raises(ValueError, match='finite'):
        draw_policy_overlay(image, 1.0, float('nan'), (0.0, 0.0))


def test_shadow_metrics_empty_normal_and_error():
    metrics = ShadowMetrics()
    assert metrics.summary()['sample_count'] == 0
    metrics.add((0.2, 0.1), (0.1, -0.1))
    summary = metrics.summary()
    assert summary['linear_mae_mps'] == pytest.approx(0.1)
    assert summary['angular_p95_radps'] == pytest.approx(0.2)
    with pytest.raises(ValueError):
        metrics.add((float('nan'), 0.0), (0.0, 0.0))


def _write_dataset(directory, count=25):
    images = directory / 'images'
    images.mkdir(parents=True)
    records = []
    for index in range(count):
        image = np.full((12, 16, 3), index * 5, dtype=np.uint8)
        name = f'images/{index}.jpg'
        assert cv2.imwrite(str(directory / name), image)
        records.append(
            {
                'image': name,
                'goal_distance_m': 1.0 + index / 10,
                'goal_bearing_rad': -0.4 + index / 50,
                'teacher_linear_x': 0.2,
                'teacher_angular_z': -0.1 + index / 100,
            }
        )
    (directory / 'samples.jsonl').write_text(
        ''.join(json.dumps(record) + '\n' for record in records),
        encoding='utf-8',
    )


def test_dataset_loading_training_and_model_round_trip(tmp_path):
    _write_dataset(tmp_path)
    inputs, targets = load_training_data(tmp_path, 4, 3)
    assert inputs.shape == (25, 39)
    assert targets.shape == (25, 2)
    training, validation = validation_indices(len(inputs))
    assert len(training) == 20
    assert len(validation) == 5
    model, report = train_policy(inputs, targets, 8, 20)
    assert report['sample_count'] == 25
    model_path = tmp_path / 'policy.yml'
    model.save(str(model_path))
    model_path.with_suffix('.json').write_text(
        json.dumps(
            {
                'image_width': 4,
                'image_height': 3,
                'max_goal_distance': 10.0,
                'max_linear_speed': 0.35,
                'max_angular_speed': 0.8,
            }
        ),
        encoding='utf-8',
    )
    policy = OpenCvVisualPolicy(model_path)
    prediction = policy.predict(
        np.zeros((12, 16, 3), dtype=np.uint8), 2.0, 0.1
    )
    assert -0.35 <= prediction[0] <= 0.35
    assert -0.8 <= prediction[1] <= 0.8


def test_dataset_loader_rejects_short_and_unsafe_records(tmp_path):
    _write_dataset(tmp_path, count=2)
    with pytest.raises(ValueError, match='at least 20'):
        load_training_data(tmp_path)
    (tmp_path / 'samples.jsonl').write_text(
        json.dumps(
            {
                'image': '../outside.jpg',
                'goal_distance_m': 1.0,
                'goal_bearing_rad': 0.0,
                'teacher_linear_x': 0.1,
                'teacher_angular_z': 0.0,
            }
        ),
        encoding='utf-8',
    )
    with pytest.raises(ValueError, match='invalid dataset record'):
        load_training_data(tmp_path)


def test_policy_loader_reports_corrupt_model_and_metadata(tmp_path):
    """Model loading turns backend parse failures into clear errors."""
    model = tmp_path / 'bad.yml'
    model.write_text('not an OpenCV model', encoding='utf-8')
    model.with_suffix('.json').write_text('{bad json', encoding='utf-8')
    with pytest.raises(ValueError, match='metadata'):
        OpenCvVisualPolicy(model)
    model.with_suffix('.json').write_text(
        json.dumps(
            {
                'image_width': 4,
                'image_height': 3,
                'max_goal_distance': 10,
                'max_linear_speed': 0.35,
                'max_angular_speed': 0.8,
            }
        ),
        encoding='utf-8',
    )
    with pytest.raises(ValueError, match='load visual policy'):
        OpenCvVisualPolicy(model)


def test_training_rejects_bad_shapes_and_counts():
    with pytest.raises(ValueError, match='two-column'):
        train_policy(np.zeros((20, 3)), np.zeros((20, 1)))
    with pytest.raises(ValueError, match='counts'):
        train_policy(np.zeros((21, 3)), np.zeros((20, 2)))
    with pytest.raises(ValueError, match='finite'):
        train_policy(
            np.full((20, 3), float('nan')), np.zeros((20, 2))
        )
    with pytest.raises(ValueError, match='positive'):
        train_policy(np.zeros((20, 3)), np.zeros((20, 2)), hidden_units=0)


def test_training_main_writes_loadable_artifacts(tmp_path):
    """CLI saves the model, matching metadata and a metric report."""
    dataset = tmp_path / 'dataset'
    _write_dataset(dataset)
    model = tmp_path / 'output' / 'policy.yml'
    visual_policy_train.main(
        [
            '--dataset', str(dataset),
            '--model', str(model),
            '--image-width', '4',
            '--image-height', '3',
            '--hidden-units', '8',
            '--maximum-iterations', '10',
        ]
    )
    assert model.is_file()
    assert model.with_suffix('.json').is_file()
    report = json.loads(
        (model.parent / 'policy_report.json').read_text(encoding='utf-8')
    )
    assert report['sample_count'] == 25
    assert OpenCvVisualPolicy(model).metadata['control_authority'] == 'shadow_only'


def test_training_main_rejects_nonpositive_limits():
    """CLI rejects invalid dimensions before reading a dataset."""
    with pytest.raises(SystemExit) as error:
        visual_policy_train.main(
            ['--dataset', 'missing', '--model', 'missing', '--image-width', '0']
        )
    assert error.value.code == 2
