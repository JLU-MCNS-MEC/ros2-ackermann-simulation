#!/usr/bin/env python3
"""Generate the deterministic occupancy map for complex_static.sdf."""

from __future__ import annotations

import math
from pathlib import Path


RESOLUTION = 0.05
ORIGIN_X = -15.0
ORIGIN_Y = -10.0
WIDTH = 600
HEIGHT = 400

# Axis-aligned rectangles shared with the Gazebo world. Coordinates are
# inclusive world-space bounds in metres.
OBSTACLES = (
    (-15.0, -14.7, -10.0, 10.0),
    (14.7, 15.0, -10.0, 10.0),
    (-15.0, 15.0, -10.0, -9.7),
    (-15.0, 15.0, 9.7, 10.0),
    (-10.0, -4.0, -4.5, -3.5),
    (-2.5, -1.5, -4.5, 2.5),
    (-1.5, 5.5, 3.5, 4.5),
    (6.5, 7.5, -3.5, 3.5),
    (7.0, 12.0, 5.5, 6.5),
    (-11.6, -10.4, 1.4, 2.6),
    (2.4, 3.6, 6.4, 7.6),
    (10.4, 11.6, -2.6, -1.4),
)


def _occupancy_grid() -> bytearray:
    """Return a top-left-origin PGM raster for the configured geometry."""
    pixels = bytearray([254]) * (WIDTH * HEIGHT)
    for row in range(HEIGHT):
        y = ORIGIN_Y + (HEIGHT - row - 0.5) * RESOLUTION
        for column in range(WIDTH):
            x = ORIGIN_X + (column + 0.5) * RESOLUTION
            if any(
                x_min <= x <= x_max and y_min <= y <= y_max
                for x_min, x_max, y_min, y_max in OBSTACLES
            ):
                pixels[row * WIDTH + column] = 0
    return pixels


def _write_pgm(path: Path) -> None:
    """Write the occupancy grid as a compact binary PGM image."""
    if not math.isclose(WIDTH * RESOLUTION, 30.0):
        raise ValueError('map width must remain 30 metres')
    path.parent.mkdir(parents=True, exist_ok=True)
    header = f'P5\n# complex_static.sdf at {RESOLUTION} m/pixel\n'
    header += f'{WIDTH} {HEIGHT}\n255\n'
    path.write_bytes(header.encode('ascii') + _occupancy_grid())


if __name__ == '__main__':
    _write_pgm(Path(__file__).parents[1] / 'maps' / 'complex_static.pgm')
