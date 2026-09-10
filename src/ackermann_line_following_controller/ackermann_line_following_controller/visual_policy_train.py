"""Train a compact image-and-goal behavior-cloning baseline with OpenCV."""

from __future__ import annotations

import argparse
import json
import math
from pathlib import Path

import cv2
import numpy as np

from .visual_policy import encode_policy_input, encode_policy_target


def load_training_data(
    dataset_directory: str | Path,
    image_width: int = 48,
    image_height: int = 27,
    max_goal_distance: float = 10.0,
    max_linear_speed: float = 0.35,
    max_angular_speed: float = 0.8,
) -> tuple[np.ndarray, np.ndarray]:
    """Load and validate a recorder dataset into model matrices."""
    directory = Path(dataset_directory).resolve()
    manifest = directory / 'samples.jsonl'
    if not manifest.is_file():
        raise FileNotFoundError(f'dataset manifest not found: {manifest}')
    inputs = []
    targets = []
    for line_number, line in enumerate(
        manifest.read_text(encoding='utf-8').splitlines(), start=1
    ):
        if not line.strip():
            continue
        try:
            record = json.loads(line)
            image_path = (directory / record['image']).resolve()
            image_path.relative_to(directory)
            image = cv2.imread(str(image_path), cv2.IMREAD_COLOR)
            if image is None:
                raise ValueError(f'unreadable image: {image_path}')
            inputs.append(
                encode_policy_input(
                    image,
                    float(record['goal_distance_m']),
                    float(record['goal_bearing_rad']),
                    image_width,
                    image_height,
                    max_goal_distance,
                ).reshape(-1)
            )
            targets.append(
                encode_policy_target(
                    float(record['teacher_linear_x']),
                    float(record['teacher_angular_z']),
                    max_linear_speed,
                    max_angular_speed,
                )
            )
        except (
            KeyError, TypeError, ValueError, json.JSONDecodeError
        ) as error:
            raise ValueError(
                f'invalid dataset record at line {line_number}: {error}'
            ) from error
    if len(inputs) < 20:
        raise ValueError('at least 20 valid samples are required')
    return np.asarray(inputs, dtype=np.float32), np.asarray(
        targets, dtype=np.float32
    )


def validation_indices(sample_count: int) -> tuple[np.ndarray, np.ndarray]:
    """Return deterministic interleaved train and validation indices."""
    if sample_count < 20:
        raise ValueError('at least 20 samples are required')
    all_indices = np.arange(sample_count)
    validation = all_indices[4::5]
    training = np.setdiff1d(all_indices, validation, assume_unique=True)
    return training, validation


def train_policy(
    inputs: np.ndarray,
    targets: np.ndarray,
    hidden_units: int = 64,
    maximum_iterations: int = 300,
) -> tuple[object, dict[str, float | int]]:
    """Train an OpenCV MLP and report held-out command errors."""
    if inputs.ndim != 2 or targets.ndim != 2 or targets.shape[1] != 2:
        raise ValueError('inputs and two-column targets must be matrices')
    if len(inputs) != len(targets):
        raise ValueError('input and target sample counts must match')
    if not np.isfinite(inputs).all() or not np.isfinite(targets).all():
        raise ValueError('training matrices must be finite')
    if hidden_units <= 0 or maximum_iterations <= 0:
        raise ValueError('training limits must be positive')
    training, validation = validation_indices(len(inputs))
    cv2.setRNGSeed(7)
    model = cv2.ml.ANN_MLP_create()
    model.setLayerSizes(
        np.asarray(
            [inputs.shape[1], hidden_units, max(8, hidden_units // 2), 2],
            dtype=np.int32,
        )
    )
    model.setActivationFunction(cv2.ml.ANN_MLP_SIGMOID_SYM, 1.0, 1.0)
    model.setTrainMethod(cv2.ml.ANN_MLP_RPROP, 0.01)
    model.setTermCriteria(
        (cv2.TERM_CRITERIA_MAX_ITER | cv2.TERM_CRITERIA_EPS,
         maximum_iterations, 1.0e-5)
    )
    trained = model.train(
        inputs[training], cv2.ml.ROW_SAMPLE, targets[training]
    )
    if not trained:
        raise RuntimeError('OpenCV failed to train the visual policy')
    _, prediction = model.predict(inputs[validation])
    absolute_error = np.abs(prediction - targets[validation])
    return model, {
        'sample_count': int(len(inputs)),
        'training_sample_count': int(len(training)),
        'validation_sample_count': int(len(validation)),
        'normalized_linear_mae': float(np.mean(absolute_error[:, 0])),
        'normalized_angular_mae': float(np.mean(absolute_error[:, 1])),
        'normalized_linear_p95': float(
            np.percentile(absolute_error[:, 0], 95)
        ),
        'normalized_angular_p95': float(
            np.percentile(absolute_error[:, 1], 95)
        ),
    }


def main(args=None) -> None:
    """Train one reproducible baseline and save model, metadata and report."""
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--dataset', required=True)
    parser.add_argument('--model', required=True)
    parser.add_argument('--image-width', type=int, default=48)
    parser.add_argument('--image-height', type=int, default=27)
    parser.add_argument('--hidden-units', type=int, default=64)
    parser.add_argument('--maximum-iterations', type=int, default=300)
    parser.add_argument('--max-goal-distance', type=float, default=10.0)
    parser.add_argument('--max-linear-speed', type=float, default=0.35)
    parser.add_argument('--max-angular-speed', type=float, default=0.8)
    options = parser.parse_args(args)
    positive = (
        options.image_width,
        options.image_height,
        options.hidden_units,
        options.maximum_iterations,
        options.max_goal_distance,
        options.max_linear_speed,
        options.max_angular_speed,
    )
    if not all(
        math.isfinite(float(value)) and value > 0 for value in positive
    ):
        parser.error('all sizes, limits and iteration counts must be positive')
    inputs, targets = load_training_data(
        options.dataset,
        options.image_width,
        options.image_height,
        options.max_goal_distance,
        options.max_linear_speed,
        options.max_angular_speed,
    )
    model, report = train_policy(
        inputs, targets, options.hidden_units, options.maximum_iterations
    )
    model_path = Path(options.model)
    model_path.parent.mkdir(parents=True, exist_ok=True)
    model.save(str(model_path))
    metadata = {
        'format_version': 1,
        'backend': 'opencv_ann_mlp',
        'image_width': options.image_width,
        'image_height': options.image_height,
        'max_goal_distance': options.max_goal_distance,
        'max_linear_speed': options.max_linear_speed,
        'max_angular_speed': options.max_angular_speed,
        'training_dataset': str(Path(options.dataset).resolve()),
        'control_authority': 'shadow_only',
    }
    model_path.with_suffix('.json').write_text(
        json.dumps(metadata, indent=2), encoding='utf-8'
    )
    report_path = model_path.with_name(f'{model_path.stem}_report.json')
    report_path.write_text(json.dumps(report, indent=2), encoding='utf-8')
    print(json.dumps(report, indent=2), flush=True)


if __name__ == '__main__':
    main()
