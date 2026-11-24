"""Occupancy-grid denoising and cached known-free connectivity for HGVD."""
from collections import deque
from functools import lru_cache

import numpy as np
from scipy import ndimage

STEPS = ((1, 0), (-1, 0), (0, 1), (0, -1))


class Connectivity:
    def __init__(self, free):
        self.free = free
        self.height, self.width = free.shape
        self._cached_line = lru_cache(maxsize=8192)(self._check_line)

    def contains(self, cell):
        x, y = cell
        return 0 <= x < self.width and 0 <= y < self.height and self.free[y, x]

    def neighbors(self, cell):
        x, y = cell
        for dx, dy in STEPS:
            neighbor = (x + dx, y + dy)
            if self.contains(neighbor):
                yield neighbor

    def direct(self, first, second):
        # Normalize whole endpoints so reversed queries use the same segment.
        return self._cached_line(*sorted((first, second)))

    def _check_line(self, first, second):
        if not (self.contains(first) and self.contains(second)):
            return False
        x, y = first
        end_x, end_y = second
        dx, dy = abs(end_x - x), -abs(end_y - y)
        step_x, step_y = (1 if x < end_x else -1), (1 if y < end_y else -1)
        error = dx + dy
        while (x, y) != second:
            old_x, old_y = x, y
            twice_error = 2 * error
            if twice_error >= dy:
                x += step_x
                error += dy
            if twice_error <= dx:
                y += step_y
                error += dx
            if not self.contains((x, y)):
                return False
            if x != old_x and y != old_y:
                if not (self.contains((x, old_y)) and self.contains((old_x, y))):
                    return False
        return True

    def grid_distances(self, start):
        distances = {start: 0}
        pending = deque([start])
        while pending:
            current = pending.popleft()
            for neighbor in self.neighbors(current):
                if neighbor not in distances:
                    distances[neighbor] = distances[current] + 1
                    pending.append(neighbor)
        return distances


def denoise(grid, previous, previous_clean):
    """Fill isolated unknowns near free space for GVD generation only."""
    if previous is None or previous.shape != grid.shape:
        changed = np.ones(grid.shape, dtype=bool)
        clean = grid.copy()
    else:
        changed = ndimage.maximum_filter(grid != previous, size=5)
        clean = previous_clean.copy()
    support = ndimage.convolve((grid == 0).astype(np.uint8), np.ones((5, 5)), mode='constant')
    clean[changed] = grid[changed]
    clean[changed & (grid == -1) & (support >= 3)] = 0
    return clean
