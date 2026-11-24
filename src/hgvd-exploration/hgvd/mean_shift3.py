"""Connectivity-constrained Mean Shift for HGVD samples."""
from math import dist

import numpy as np
from scipy.spatial import cKDTree


def mean_shift(samples, links, bandwidth):
    if not samples:
        return []
    points = np.asarray(samples)
    tree = cKDTree(points)
    modes = {}
    for seed in samples:
        center = seed
        intensity = 0
        for _ in range(50):
            neighbors = [i for i in tree.query_ball_point(center, bandwidth)
                         if links.direct(center, samples[i])]
            if not neighbors:
                break
            mean = tuple(int(round(v)) for v in points[neighbors].mean(axis=0))
            intensity = len(neighbors)
            if mean == center or not links.direct(center, mean):
                break
            if _ < 49:
                center = mean
        if intensity:
            modes[center] = max(modes.get(center, 0), intensity)

    centers = []
    for mode in sorted(modes, key=lambda cell: (-modes[cell], cell)):
        # Suppress every reachable nearby mode. Modes separated by walls
        # stay distinct, even when their Euclidean distance is small.
        if not any(dist(mode, center) <= bandwidth and links.direct(mode, center)
                   for center in centers):
            centers.append(mode)
    return centers
