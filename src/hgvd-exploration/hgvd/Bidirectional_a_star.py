"""Full-resolution bidirectional A* for connecting HGVD components.

Adapted from the original 2D Bidirectional A* by huiming zhou.
"""
from heapq import heappop, heappush
from math import inf


def bidirectional_astar(links, start, goal):
    """Return a shortest 4-connected grid path length or infinity."""
    if not (links.contains(start) and links.contains(goal)):
        return inf
    if start == goal:
        return 0

    def potential(cell):
        # Twice the consistent, balanced A* potential. Integer keys keep
        # comparisons exact when applying the bidirectional stop condition.
        return (abs(cell[0] - goal[0]) + abs(cell[1] - goal[1])
                - abs(cell[0] - start[0]) - abs(cell[1] - start[1]))

    costs = [{start: 0}, {goal: 0}]
    queues = [[(potential(start), 0, start)], [(-potential(goal), 0, goal)]]
    best = inf
    while queues[0] and queues[1]:
        if queues[0][0][0] + queues[1][0][0] >= 2 * best:
            break
        side = 0 if queues[0][0] <= queues[1][0] else 1
        _, length, cell = heappop(queues[side])
        if length != costs[side][cell]:
            continue
        for neighbor in links.neighbors(cell):
            new_length = length + 1
            if new_length < costs[side].get(neighbor, inf):
                costs[side][neighbor] = new_length
                best = min(best, new_length + costs[1 - side].get(neighbor, inf))
                sign = 1 if side == 0 else -1
                heappush(queues[side], (2 * new_length + sign * potential(neighbor),
                                        new_length, neighbor))
    return best
