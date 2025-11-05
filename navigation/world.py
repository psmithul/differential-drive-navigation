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
        ax, ay = start[:2]
        bx, by = end[:2]
        if not self.free(start, radius) or not self.free(end, radius):
            return False
        dx, dy = bx - ax, by - ay
        length_squared = dx*dx + dy*dy
        if length_squared == 0:
            return True
        for x0, y0, x1, y1 in self.obstacles:
            enter, leave = 0.0, 1.0
            for position, delta, low, high in ((ax, dx, x0, x1), (ay, dy, y0, y1)):
                if delta == 0:
                    if position < low or position > high:
                        enter, leave = 1.0, 0.0
                        break
                else:
                    t0, t1 = sorted(((low - position) / delta, (high - position) / delta))
                    enter, leave = max(enter, t0), min(leave, t1)
            if enter <= leave:
                return False
            for x, y in ((x0, y0), (x0, y1), (x1, y0), (x1, y1)):
                t = max(0.0, min(1.0, ((x - ax)*dx + (y - ay)*dy) / length_squared))
                if (x - ax - t*dx)**2 + (y - ay - t*dy)**2 <= radius**2:
                    return False
        return True

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
        def connect(endpoint):
            cx, cy = cell(endpoint)
            if (cx, cy) not in occupied and self.segment_free(endpoint, point((cx, cy)), radius):
                return cx, cy
            nearby = [(x, y) for x in range(max(0, cx-2), min(nx, cx+3))
                      for y in range(max(0, cy-2), min(ny, cy+3)) if (x, y) not in occupied]
            nearby.sort(key=lambda node: (point(node)[0] - endpoint[0])**2 +
                                        (point(node)[1] - endpoint[1])**2)
            for node in nearby:
                if self.segment_free(endpoint, point(node), radius):
                    return node
            raise ValueError("No clear connection from endpoint to planning grid")

        first, last = connect(start), connect(goal)
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
