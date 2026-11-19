#!/usr/bin/env python3
"""
TATARO Therapy — Shared target-selection logic.

Used by both the real FSM (fsm_node.py) and standalone diagnostics (e.g.
level1_grid_test.py) so "which pose is next for this repetition" lives in
exactly one place -- avoids the FSM and a diagnostic script quietly drifting
to different selection behavior over time.
"""

import random


class TargetSelector:
    """
    Selects the next target pose for one level.

    - Single-pose level (`pose_mm_deg` key): always returns that same pose.
    - Grid level (`grid` key, sharing one `fixed` orientation/X): returns
      points from a shuffled "bag" of grid indices, refilled and reshuffled
      only once emptied -- every point is used exactly once per cycle
      through the grid before any repeats.
    """

    def __init__(self, level: dict):
        self.level = level
        self._bag: list = []

    def next(self) -> tuple:
        """Return (x_mm, y_mm, z_mm, roll_deg, pitch_deg, yaw_deg, label)."""
        level = self.level
        if 'grid' in level:
            grid = level['grid']
            if not self._bag:
                self._bag = list(range(len(grid)))
                random.shuffle(self._bag)
            idx = self._bag.pop()
            point = grid[idx]
            fixed = level['fixed']
            label = f"{level['name']} / {point.get('name', f'grid[{idx}]')}"
            return (fixed['x_mm'], point['y_mm'], point['z_mm'],
                    fixed['roll_deg'], fixed['pitch_deg'], fixed['yaw_deg'], label)

        x_mm, y_mm, z_mm, roll, pitch, yaw = level['pose_mm_deg']
        return x_mm, y_mm, z_mm, roll, pitch, yaw, level['name']

    def reset(self) -> None:
        """Discard the current bag -- the next `next()` call starts a fresh shuffle."""
        self._bag = []
