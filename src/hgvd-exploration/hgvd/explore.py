"""Select reachable exploration viewpoints using the HGVD topology."""
from collections import defaultdict
from contextlib import nullcontext
from math import atan2, ceil, dist, pi

import numpy as np
from scipy import ndimage
from scipy.spatial import cKDTree

from .gvd import CROSS, frontier_cells, sample_gvd
from .mean_shift3 import mean_shift
from .topology import build_topology, shortest_paths
from .utils import Connectivity, denoise


class Planner:
    def __init__(self, local_radius_m, local_stride, global_stride, bandwidth_m,
                 clearance_m, frontier_spacing_m, *, measure=nullcontext):
        self.measure = measure
        self.local_radius_m = local_radius_m
        self.local_stride = local_stride
        self.global_stride = global_stride
        self.bandwidth_m = bandwidth_m
        self.clearance_m = clearance_m
        self.frontier_spacing_m = frontier_spacing_m
        self.grid = None
        self.sample_origin = None
        self.samples = []
        self.centers = []
        self.graph = {}

    def update_map(self, grid, resolution):
        with self.measure('map_denoise'):
            self.clean = denoise(grid, self.grid, self.clean if self.grid is not None else None)
        self.grid = grid
        self.resolution = resolution
        with self.measure('map_connectivity'):
            traversable = grid == 0
            if np.any(grid == 100):
                self.clearance = ndimage.distance_transform_edt(grid != 100) * resolution
            else:
                self.clearance = np.full(grid.shape, np.inf)
            traversable &= self.clearance + 1e-6 >= self.clearance_m
            self.components, _ = ndimage.label(traversable, structure=CROSS)
        self.sample_origin = None

    def update_topology(self, robot):
        x, y = robot
        height, width = self.grid.shape
        if not (0 <= x < width and 0 <= y < height):
            return False
        component = self.components[y, x]
        if component == 0:
            return False
        if (self.sample_origin is not None and component == self.component
                and dist(robot, self.sample_origin) < self.local_stride):
            return True
        self.sample_origin = robot
        self.component = component
        self.reachable = self.components == component
        self.links = Connectivity(self.reachable)
        with self.measure('gvd_sampling'):
            self.samples = sample_gvd(self.grid, self.clean, self.reachable, robot,
                                      ceil(self.local_radius_m / self.resolution),
                                      self.local_stride, self.global_stride)
        cells = [sample.cell for sample in self.samples]
        with self.measure('mean_shift'):
            self.centers = mean_shift(cells, self.links, self.bandwidth_m / self.resolution)
        with self.measure('topology_graph'):
            self.graph = build_topology(cells, self.centers, self.links)
        with self.measure('frontier_detection'):
            self.frontiers = frontier_cells(self.grid, self.reachable)
            self.wall_neighbor = ndimage.binary_dilation(self.grid == 100, structure=np.ones((5, 5)))
        return True

    def viewpoints(self, robot, excluded, min_distance_m=0.0, *, robot_position=None):
        """Choose representatives of GVD units and of frontiers outside them."""
        spacing = max(1, round(self.frontier_spacing_m / self.resolution))
        robot_position = robot if robot_position is None else robot_position
        min_distance = min_distance_m / self.resolution
        groups = defaultdict(list)
        tree = cKDTree([sample.cell for sample in self.samples]) if self.samples else None
        points = [tuple(map(int, row)) for row in np.argwhere(self.frontiers)[:, ::-1]]
        if tree is not None and points:
            count = min(4, len(self.samples))
            distances, indices = tree.query(points, k=list(range(1, count + 1)))
        for i, cell in enumerate(points):
            if cell == robot or cell in excluded or dist(robot_position, cell) < min_distance:
                continue
            x, y = cell
            group = ('grid', x // spacing, y // spacing)
            if tree is not None:
                for distance, index in zip(distances[i], indices[i]):
                    sample = self.samples[index]
                    if distance <= sample.radius and self.links.direct(sample.cell, cell):
                        group = ('gvd', index)
                        break
            groups[group].append(cell)

        for group, cells in groups.items():
            # Prefer frontiers away from walls without discarding narrow
            # doorways whose entire frontier is close to an obstacle.
            reference = self.samples[group[1]].cell if group[0] == 'gvd' else robot
            yield min(cells, key=lambda p: (self.wall_neighbor[p[1], p[0]],
                                            dist(reference, p), dist(robot, p), p))

    def scores(self, robot, yaw, targets):
        starts = {cell: dist(robot, cell) for cell in self.graph if self.links.direct(robot, cell)}
        routes = shortest_paths(self.graph, starts)
        # Only attach to nearby graph vertices. A frontier outside the
        # topology can still be ranked using the known-free grid.
        attach_radius = self.bandwidth_m / self.resolution
        grid_distances = None
        for target in targets:
            angle = abs((atan2(target[1] - robot[1], target[0] - robot[0]) - yaw + pi)
                        % (2 * pi) - pi)
            if self.links.direct(robot, target):
                distance = dist(robot, target) * (1.0 + 0.1 * angle)
            else:
                distance = min(
                    (length * (1 + 0.1 * (hops + 1)) + tail * (1 + angle)
                     for cell, (length, hops) in routes.items()
                     if (tail := dist(cell, target)) <= attach_radius
                     and self.links.direct(cell, target)),
                    default=float('inf'),
                )
                if not np.isfinite(distance):
                    if grid_distances is None:
                        grid_distances = self.links.grid_distances(robot)
                    distance = grid_distances.get(target, float('inf')) * (1.0 + 0.1 * angle)
            yield target, distance * self.resolution
