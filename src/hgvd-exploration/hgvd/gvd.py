"""Hierarchical GVD sampling on a planar occupancy grid."""
from dataclasses import dataclass

import numpy as np
from scipy import ndimage
from scipy.spatial import cKDTree

Cell = tuple[int, int]  # (column, row)
CROSS = ndimage.generate_binary_structure(2, 1)


@dataclass(frozen=True)
class Sample:
    cell: Cell
    radius: float


def sample_gvd(grid, clean, reachable, robot, local_radius, local_stride, global_stride):
    """Find near-equidistant obstacle sites and keep nonredundant GVD centers.

    Strides are spatial, so a small map does not depend on the random ordering
    of its free cells. Unknown-only maps need no expanding obstacle search.
    """
    obstacles = np.argwhere(grid == 100)[:, ::-1]
    if len(obstacles) < 2:
        return []

    tree = cKDTree(obstacles)
    height, width = grid.shape
    radius_squared = local_radius ** 2
    cover = np.zeros(grid.shape, dtype=bool)
    candidates = []
    for stride, local in ((local_stride, True), (global_stride, False)):
        if local:
            x0, y0 = max(0, robot[0] - local_radius), max(0, robot[1] - local_radius)
            x1, y1 = min(width, robot[0] + local_radius + 1), min(height, robot[1] + local_radius + 1)
        else:
            x0, y0, x1, y1 = 0, 0, width, height
        x, y = np.meshgrid(np.arange(x0, x1, stride), np.arange(y0, y1, stride))
        inside = (x - robot[0]) ** 2 + (y - robot[1]) ** 2 <= radius_squared
        eligible = reachable[y, x] & (inside == local)
        coords = np.column_stack((x[eligible], y[eligible]))
        if not len(coords):
            continue
        nearest, _ = tree.query(coords, p=1)
        clearance, _ = tree.query(coords)
        for (x, y), distance, radius in zip(coords, nearest, clearance):
            nearby = obstacles[tree.query_ball_point((x, y), distance + 2, p=1)]
            # Keep separated obstacle sites, rather than several pixels on one
            # nearby wall segment. Limit the radius to the nearest site.
            separation = np.ptp(nearby, axis=0)
            if max(separation) <= 5:
                continue
            if radius >= 2.0:
                candidates.append(Sample((int(x), int(y)), float(radius)))

    samples = []
    for sample in sorted(candidates, key=lambda s: (-s.radius, s.cell)):
        x, y = sample.cell
        if cover[y, x]:
            continue
        samples.append(sample)
        r = int(np.ceil(sample.radius))
        x0, x1 = max(0, x - r), min(width, x + r + 1)
        y0, y1 = max(0, y - r), min(height, y + r + 1)
        yy, xx = np.ogrid[y0:y1, x0:x1]
        # Denoised free space may extend the geometric unit, but only raw
        # reachable cells can become navigation goals or suppress candidates.
        unit = (xx - x) ** 2 + (yy - y) ** 2 <= sample.radius ** 2
        known_free = clean[y0:y1, x0:x1] == 0
        visible_labels, _ = ndimage.label(unit & known_free, structure=CROSS)
        cover[y0:y1, x0:x1] |= visible_labels == visible_labels[y - y0, x - x0]
    return samples


def frontier_cells(grid, reachable):
    """Free cells with a traversable side facing unknown space."""
    return reachable & ndimage.binary_dilation(grid == -1, structure=CROSS)
