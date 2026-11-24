"""Construct and search the weighted HGVD topology graph."""
from heapq import heapify, heappop, heappush
from math import dist, inf

import numpy as np
from scipy.spatial import cKDTree

from .Bidirectional_a_star import bidirectional_astar


def add_edge(graph, first, second, length):
    graph[first][second] = length
    graph[second][first] = length


def build_topology(samples, centers, links, neighbor_count=10):
    # Include sampled points so the graph can connect modes along the GVD
    # even when the straight segment between the modes is obstructed.
    cells = list(dict.fromkeys([*centers, *samples]))
    graph = {cell: {} for cell in cells}
    if len(cells) < 2:
        return graph
    tree = cKDTree(cells)
    _, near = tree.query(cells, k=min(neighbor_count + 1, len(cells)))
    for i, cell in enumerate(cells):
        for j in near[i]:
            other = cells[j]
            if cell != other and links.direct(cell, other):
                add_edge(graph, cell, other, dist(cell, other))

    while True:
        reached = shortest_paths(graph, {cells[0]: 0})
        if len(reached) == len(cells):
            return graph
        remaining = [cell for cell in cells if cell not in reached]
        connected = list(reached)
        tree = cKDTree(connected)
        distances, indices = tree.query(remaining)
        best = (inf, None, None)
        for i in np.argsort(distances)[:neighbor_count]:
            if distances[i] >= best[0]:
                break
            first, second = remaining[i], connected[indices[i]]
            length = bidirectional_astar(links, first, second)
            if length < best[0]:
                best = (length, first, second)
        if best[1] is None:
            return graph
        add_edge(graph, best[1], best[2], best[0])


def shortest_paths(graph, starts):
    """Find weighted route distances and hop counts from a virtual start."""
    costs = {cell: (length, 0) for cell, length in starts.items()}
    queue = [(length, 0, cell) for cell, length in starts.items()]
    heapify(queue)
    while queue:
        length, hops, cell = heappop(queue)
        if costs[cell] != (length, hops):
            continue
        for neighbor, edge in graph[cell].items():
            cost = (length + edge, hops + 1)
            if cost < costs.get(neighbor, (inf, 0)):
                costs[neighbor] = cost
                heappush(queue, (*cost, neighbor))
    return costs
