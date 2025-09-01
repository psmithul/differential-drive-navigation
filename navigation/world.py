from dataclasses import dataclass
import heapq
import math

import numpy as np


@dataclass
class World:
    width: float = 12.0
    height: float = 10.0
    resolution: float = 0.25
    obstacles: tuple = ((3.5, 0.0, 4.0, 6.0), (7.5, 4.0, 8.0, 10.0))
    start: tuple = (1.0, 1.0, 0.0)
    goal: tuple = (11.0, 9.0)

    def free(self, point, radius):
        x, y = point[:2]
        if not (radius < x < self.width - radius and radius < y < self.height - radius):
            return False
        for x0, y0, x1, y1 in self.obstacles:
            dx = max(x0 - x, 0.0, x - x1)
            dy = max(y0 - y, 0.0, y - y1)
            if dx * dx + dy * dy <= radius * radius:
                return False
        return True

    def segment_free(self, start, end, radius):
        start, end = np.asarray(start)[:2], np.asarray(end)[:2]
        count = max(1, math.ceil(np.linalg.norm(end - start) / 0.04))
        return all(self.free(start + (end - start) * i / count, radius)
                   for i in range(count + 1))

    def plan(self, start, goal, radius=0.45):
        if not self.free(start, radius) or not self.free(goal, radius):
            raise ValueError("Start or goal is inside the inflated obstacles")
        step = self.resolution
        nx, ny = round(self.width / step), round(self.height / step)

        def cell(point):
            return int(point[0] / step), int(point[1] / step)

        def point(node):
            return ((node[0] + 0.5) * step, (node[1] + 0.5) * step)

        occupied = {(x, y) for x in range(nx) for y in range(ny)
                    if not self.free(point((x, y)), radius)}
        first, last = cell(start), cell(goal)
        if first in occupied or last in occupied:
            raise ValueError("Start or goal grid cell is blocked")
        queue = [(0.0, first)]
        cost, parent = {first: 0.0}, {}
        visited = set()
        while queue:
            _, node = heapq.heappop(queue)
            if node in visited:
                continue
            if node == last:
                nodes = [node]
                while node != first:
                    node = parent[node]
                    nodes.append(node)
                path = [tuple(start[:2])] + [point(n) for n in reversed(nodes)] + [tuple(goal)]
                if not all(self.segment_free(a, b, radius) for a, b in zip(path, path[1:])):
                    raise ValueError("Grid path cannot connect to the requested endpoints")
                return self.smooth(path, radius)
            visited.add(node)
            for dx, dy in ((1, 0), (-1, 0), (0, 1), (0, -1),
                           (1, 1), (1, -1), (-1, 1), (-1, -1)):
                next_node = node[0] + dx, node[1] + dy
                if not (0 <= next_node[0] < nx and 0 <= next_node[1] < ny):
                    continue
                if next_node in occupied:
                    continue
                if dx and dy and ((node[0] + dx, node[1]) in occupied or
                                  (node[0], node[1] + dy) in occupied):
                    continue
                if not self.segment_free(point(node), point(next_node), radius):
                    continue
                candidate = cost[node] + math.hypot(dx, dy)
                if candidate < cost.get(next_node, math.inf):
                    cost[next_node] = candidate
                    parent[next_node] = node
                    heuristic = math.hypot(next_node[0] - last[0], next_node[1] - last[1])
                    heapq.heappush(queue, (candidate + heuristic, next_node))
        raise ValueError("No route connects start and goal")

    def smooth(self, path, radius):
        result, index = [path[0]], 0
        while index < len(path) - 1:
            target = len(path) - 1
            while target > index + 1 and not self.segment_free(path[index], path[target], radius):
                target -= 1
            result.append(path[target])
            index = target
        return result
