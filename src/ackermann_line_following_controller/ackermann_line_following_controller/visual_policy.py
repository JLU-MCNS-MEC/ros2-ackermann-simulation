"""Preprocess and run a goal-conditioned visual control policy."""

from __future__ import annotations

import json
import math
from pathlib import Path

import cv2
import numpy as np


class ShadowMetrics:
    """Accumulate visual-policy errors against a Nav2 teacher."""

    def __init__(self) -> None:
        """Initialize an empty comparison."""
        self.linear_errors: list[float] = []
        self.angular_errors: list[float] = []

    def add(
        self,
        predicted: tuple[float, float],
        teacher: tuple[float, float],
    ) -> None:
        """Add one pair of finite commands."""
        values = (*predicted, *teacher)
        if not all(math.isfinite(value) for value in values):
            raise ValueError('shadow commands must be finite')
        self.linear_errors.append(abs(predicted[0] - teacher[0]))
        self.angular_errors.append(abs(predicted[1] - teacher[1]))

    def summary(self) -> dict[str, float | int | None]:
        """Return sample count, MAE and P95 for both commands."""
        if not self.linear_errors:
            return {
                'sample_count': 0,
                'linear_mae_mps': None,
                'angular_mae_radps': None,
                'linear_p95_mps': None,
                'angular_p95_radps': None,
            }
        return {
            'sample_count': len(self.linear_errors),
            'linear_mae_mps': float(np.mean(self.linear_errors)),
            'angular_mae_radps': float(np.mean(self.angular_errors)),
            'linear_p95_mps': float(np.percentile(self.linear_errors, 95)),
            'angular_p95_radps': float(np.percentile(self.angular_errors, 95)),
        }


def normalize_angle(angle: float) -> float:
    """Wrap a finite angle to ``[-pi, pi]``."""
    if not math.isfinite(angle):
        raise ValueError('angle must be finite')
    return math.atan2(math.sin(angle), math.cos(angle))


def relative_goal(
    robot_x: float,
    robot_y: float,
    robot_yaw: float,
    goal_x: float,
    goal_y: float,
) -> tuple[float, float]:
    """Return goal distance and bearing in the robot frame."""
    values = (robot_x, robot_y, robot_yaw, goal_x, goal_y)
    if not all(math.isfinite(value) for value in values):
        raise ValueError('robot and goal poses must be finite')
    dx = goal_x - robot_x
    dy = goal_y - robot_y
    return math.hypot(dx, dy), normalize_angle(math.atan2(dy, dx) - robot_yaw)


def quaternion_to_yaw(x: float, y: float, z: float, w: float) -> float:
    """Return planar yaw from a finite, nonzero quaternion."""
    values = np.asarray((x, y, z, w), dtype=np.float64)
    if not np.isfinite(values).all():
        raise ValueError('quaternion must be finite')
    norm = float(np.linalg.norm(values))
    if norm <= 1.0e-9:
        raise ValueError('quaternion must be nonzero')
    x, y, z, w = (values / norm).tolist()
    return math.atan2(
        2.0 * (w * z + x * y),
        1.0 - 2.0 * (y * y + z * z),
    )


def encode_policy_input(
    image: np.ndarray,
    goal_distance: float,
    goal_bearing: float,
    width: int = 48,
    height: int = 27,
    max_goal_distance: float = 10.0,
) -> np.ndarray:
    """Encode a BGR image and relative goal as one normalized model row."""
    if image.ndim != 3 or image.shape[2] != 3 or image.size == 0:
        raise ValueError('image must be a nonempty BGR image')
    if width <= 0 or height <= 0:
        raise ValueError('image dimensions must be positive')
    if not math.isfinite(max_goal_distance) or max_goal_distance <= 0.0:
        raise ValueError('max_goal_distance must be positive and finite')
    if not math.isfinite(goal_distance) or goal_distance < 0.0:
        raise ValueError('goal_distance must be finite and nonnegative')
    bearing = normalize_angle(goal_bearing)
    resized = cv2.resize(image, (width, height), interpolation=cv2.INTER_AREA)
    pixels = resized.astype(np.float32).reshape(-1) / 127.5 - 1.0
    goal = np.asarray(
        [
            min(goal_distance, max_goal_distance) / max_goal_distance,
            math.sin(bearing),
            math.cos(bearing),
        ],
        dtype=np.float32,
    )
    return np.concatenate((pixels, goal)).reshape(1, -1)


def encode_policy_target(
    linear_x: float,
    angular_z: float,
    max_linear_speed: float = 0.35,
    max_angular_speed: float = 0.8,
) -> np.ndarray:
    """Normalize one teacher command to the model output range."""
    if not all(math.isfinite(value) for value in (linear_x, angular_z)):
        raise ValueError('teacher command must be finite')
    if (
        not math.isfinite(max_linear_speed)
        or not math.isfinite(max_angular_speed)
        or max_linear_speed <= 0.0
        or max_angular_speed <= 0.0
    ):
        raise ValueError('speed limits must be positive')
    return np.asarray(
        [
            np.clip(linear_x / max_linear_speed, -1.0, 1.0),
            np.clip(angular_z / max_angular_speed, -1.0, 1.0),
        ],
        dtype=np.float32,
    )


def decode_policy_output(
    prediction: np.ndarray,
    max_linear_speed: float = 0.35,
    max_angular_speed: float = 0.8,
) -> tuple[float, float]:
    """Convert a normalized model output into a bounded Twist command."""
    values = np.asarray(prediction, dtype=np.float32).reshape(-1)
    if values.size != 2 or not np.isfinite(values).all():
        raise ValueError('prediction must contain two finite values')
    if (
        not math.isfinite(max_linear_speed)
        or not math.isfinite(max_angular_speed)
        or max_linear_speed <= 0.0
        or max_angular_speed <= 0.0
    ):
        raise ValueError('speed limits must be positive')
    return (
        float(np.clip(values[0], -1.0, 1.0) * max_linear_speed),
        float(np.clip(values[1], -1.0, 1.0) * max_angular_speed),
    )


class OpenCvVisualPolicy:
    """Load and run a compact OpenCV MLP visual policy."""

    def __init__(self, model_path: str | Path) -> None:
        """Load the model and its adjacent JSON metadata."""
        self.model_path = Path(model_path)
        metadata_path = self.model_path.with_suffix('.json')
        if not self.model_path.is_file():
            raise FileNotFoundError(
                f'visual policy not found: {self.model_path}'
            )
        if not metadata_path.is_file():
            raise FileNotFoundError(
                f'policy metadata not found: {metadata_path}'
            )
        try:
            self.metadata = json.loads(
                metadata_path.read_text(encoding='utf-8')
            )
        except (OSError, json.JSONDecodeError) as error:
            raise ValueError(
                f'failed to read policy metadata: {metadata_path}'
            ) from error
        required = {
            'image_width', 'image_height', 'max_goal_distance',
            'max_linear_speed', 'max_angular_speed',
        }
        if not required <= self.metadata.keys():
            raise ValueError('policy metadata is incomplete')
        try:
            self.model = cv2.ml.ANN_MLP_load(str(self.model_path))
        except cv2.error as error:
            raise ValueError(
                f'failed to load visual policy: {self.model_path}'
            ) from error
        if self.model.empty():
            raise ValueError(
                f'failed to load visual policy: {self.model_path}'
            )

    def predict(
        self,
        image: np.ndarray,
        goal_distance: float,
        goal_bearing: float,
    ) -> tuple[float, float]:
        """Predict a bounded linear and angular command."""
        row = encode_policy_input(
            image,
            goal_distance,
            goal_bearing,
            int(self.metadata['image_width']),
            int(self.metadata['image_height']),
            float(self.metadata['max_goal_distance']),
        )
        _, prediction = self.model.predict(row)
        return decode_policy_output(
            prediction,
            float(self.metadata['max_linear_speed']),
            float(self.metadata['max_angular_speed']),
        )


def draw_policy_overlay(
    image: np.ndarray,
    goal_distance: float,
    goal_bearing: float,
    predicted: tuple[float, float],
    teacher: tuple[float, float] | None = None,
) -> np.ndarray:
    """Draw the predicted direction and numeric shadow comparison."""
    if image.ndim != 3 or image.shape[2] != 3 or image.size == 0:
        raise ValueError('image must be a nonempty BGR image')
    values = (goal_distance, goal_bearing, *predicted)
    if teacher is not None:
        values += teacher
    if not all(math.isfinite(value) for value in values):
        raise ValueError('overlay values must be finite')
    canvas = image.copy()
    height, width = canvas.shape[:2]
    origin = (width // 2, height - 24)
    scale = max(30, min(width, height) // 3)
    endpoint = (
        int(origin[0] - math.sin(goal_bearing) * scale),
        int(origin[1] - math.cos(goal_bearing) * scale),
    )
    cv2.arrowedLine(canvas, origin, endpoint, (40, 255, 255), 4, tipLength=0.2)
    lines = [
        f'GOAL {goal_distance:.2f} m  {math.degrees(goal_bearing):+.1f} deg',
        f'VISION v={predicted[0]:+.3f}  w={predicted[1]:+.3f}',
    ]
    if teacher is not None:
        lines.append(f'NAV2   v={teacher[0]:+.3f}  w={teacher[1]:+.3f}')
    for index, text in enumerate(lines):
        cv2.putText(
            canvas, text, (12, 26 + index * 24), cv2.FONT_HERSHEY_SIMPLEX,
            0.62, (0, 255, 80), 2, cv2.LINE_AA,
        )
    return canvas


def draw_status_overlay(image: np.ndarray, status: str) -> np.ndarray:
    """Draw an explicit non-driving state on a live camera frame."""
    if image.ndim != 3 or image.shape[2] != 3 or image.size == 0:
        raise ValueError('image must be a nonempty BGR image')
    if not status.strip():
        raise ValueError('status must not be empty')
    canvas = image.copy()
    cv2.putText(
        canvas, status, (12, 28), cv2.FONT_HERSHEY_SIMPLEX,
        0.65, (0, 220, 255), 2, cv2.LINE_AA,
    )
    return canvas
