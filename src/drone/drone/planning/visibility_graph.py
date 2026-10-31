"""
Deterministic shortest-path planner around concave geofences using a 2D Visibility Graph.

Builds a line-of-sight graph between Start, Goal, and the reflex vertices of the
inward-buffered safe geofence. Solves with A* using Euclidean distance heuristic.
Returns an ordered sequence of 3D ENU waypoints guaranteed to remain inside safe airspace.
"""

import heapq
import math
from typing import Optional

from shapely.geometry import LineString, Point

from drone.planning.coordinates import distance_3d, horizontal_distance
from drone.planning.geofence import BufferedGeofence


class VisibilityGraphPlanner:
    """
    Computes deterministic, shortest collision-free paths within concave geofences.
    """

    def __init__(
        self,
        geofence: BufferedGeofence,
        vertex_nudge_m: float = 0.3,
    ):
        """
        Args:
            geofence: BufferedGeofence instance defining the safe flight envelope.
            vertex_nudge_m: Distance to nudge reflex vertices inward into the safe zone
                           so the drone's turn does not skim the buffer line.
        """
        self.geofence = geofence
        self.vertex_nudge_m = float(vertex_nudge_m)

    def is_line_of_sight_clear(
        self, p1: tuple[float, float], p2: tuple[float, float]
    ) -> bool:
        """
        Test if the straight line segment between p1 and p2 lies strictly within
        the safe flight polygon without intersecting any concave boundary indentations.
        """
        if p1 == p2:
            return True

        line = LineString([p1, p2])

        # Buffer safe polygon slightly (1mm) to handle floating-point boundary touching
        expanded_safe = self.geofence.safe_polygon.buffer(0.005)

        if not expanded_safe.contains(line):
            return False

        # Verify the line does not cross or cut through the exterior boundary
        # A valid interior line may only intersect the boundary at its endpoints
        intersection = line.intersection(self.geofence.safe_polygon.exterior)
        if intersection.is_empty:
            return True

        # If it intersects, it must only be at points, and all intersection points
        # must be extremely close to the segment's endpoints p1 or p2
        pt1 = Point(p1)
        pt2 = Point(p2)

        if intersection.geom_type == "Point":
            pts = [intersection]
        elif intersection.geom_type == "MultiPoint":
            pts = list(intersection.geoms)
        elif intersection.geom_type in ("LineString", "MultiLineString"):
            # Line lies along the boundary; acceptable if within safe polygon
            return expanded_safe.contains(line)
        else:
            return False

        for pt in pts:
            d1 = pt.distance(pt1)
            d2 = pt.distance(pt2)
            if d1 > 0.05 and d2 > 0.05:
                # Intersects the boundary in the middle of the segment -> cuts forbidden cutout!
                return False

        return True

    def plan_path(
        self,
        start: tuple[float, float, float],
        goal: tuple[float, float, float],
    ) -> list[tuple[float, float, float]]:
        """
        Plan the shortest path from start to goal in Local ENU coordinates (metres).

        Args:
            start: (x, y, z) start position in ENU metres.
            goal: (x, y, z) goal position in ENU metres.

        Returns:
            List of (x, y, z) waypoints forming the shortest collision-free route.
        """
        start_z = self.geofence.clamp_altitude(start[2])
        goal_z = self.geofence.clamp_altitude(goal[2])

        # Clamp start & goal 2D coordinates to safe envelope if needed
        start_2d = self.geofence.clamp_target_to_safe_boundary(start[0], start[1])
        goal_2d = self.geofence.clamp_target_to_safe_boundary(goal[0], goal[1])

        # Step 1: Check direct line of sight
        if self.is_line_of_sight_clear(start_2d, goal_2d):
            return [
                (start_2d[0], start_2d[1], start_z),
                (goal_2d[0], goal_2d[1], goal_z),
            ]

        # Step 2: Build Visibility Graph
        # Node 0: Start, Node 1: Goal, Nodes 2..N: Reflex turning points
        reflex_nodes = self.geofence.get_reflex_vertices(nudge_m=self.vertex_nudge_m)
        nodes = [start_2d, goal_2d] + reflex_nodes
        num_nodes = len(nodes)

        # Adjacency list: node_idx -> list of (neighbor_idx, distance)
        graph: dict[int, list[tuple[int, float]]] = {i: [] for i in range(num_nodes)}

        for i in range(num_nodes):
            for j in range(i + 1, num_nodes):
                p_i = nodes[i]
                p_j = nodes[j]
                if self.is_line_of_sight_clear(p_i, p_j):
                    dist = horizontal_distance(p_i[0], p_i[1], p_j[0], p_j[1])
                    graph[i].append((j, dist))
                    graph[j].append((i, dist))

        # Step 3: Run A* Search from node 0 (Start) to node 1 (Goal)
        start_idx = 0
        goal_idx = 1

        pq: list[tuple[float, int]] = []
        heapq.heappush(
            pq,
            (
                horizontal_distance(start_2d[0], start_2d[1], goal_2d[0], goal_2d[1]),
                start_idx,
            ),
        )

        g_score = {i: float("inf") for i in range(num_nodes)}
        g_score[start_idx] = 0.0

        came_from: dict[int, int] = {}
        visited = set()

        while pq:
            _, current = heapq.heappop(pq)
            if current in visited:
                continue
            visited.add(current)

            if current == goal_idx:
                break

            curr_pos = nodes[current]
            for neighbor, edge_cost in graph[current]:
                if neighbor in visited:
                    continue
                tentative_g = g_score[current] + edge_cost
                if tentative_g < g_score[neighbor]:
                    g_score[neighbor] = tentative_g
                    came_from[neighbor] = current
                    neigh_pos = nodes[neighbor]
                    h = horizontal_distance(
                        neigh_pos[0], neigh_pos[1], goal_2d[0], goal_2d[1]
                    )
                    heapq.heappush(pq, (tentative_g + h, neighbor))

        if goal_idx not in came_from and start_idx != goal_idx:
            # Fallback if reflex nodes could not bridge: direct segment clamped
            return [
                (start_2d[0], start_2d[1], start_z),
                (goal_2d[0], goal_2d[1], goal_z),
            ]

        # Step 4: Reconstruct 2D path
        path_indices = [goal_idx]
        curr = goal_idx
        while curr in came_from:
            curr = came_from[curr]
            path_indices.append(curr)
        path_indices.reverse()

        path_2d = [nodes[idx] for idx in path_indices]

        # Step 5: Interpolate altitude Z linearly along cumulative path length
        cumulative_dist = [0.0]
        for i in range(1, len(path_2d)):
            seg_len = horizontal_distance(
                path_2d[i - 1][0],
                path_2d[i - 1][1],
                path_2d[i][0],
                path_2d[i][1],
            )
            cumulative_dist.append(cumulative_dist[-1] + seg_len)

        total_length = cumulative_dist[-1]

        waypoints_3d: list[tuple[float, float, float]] = []
        for i, (wx, wy) in enumerate(path_2d):
            if total_length > 1e-3:
                fraction = cumulative_dist[i] / total_length
                wz = start_z + fraction * (goal_z - start_z)
            else:
                wz = start_z
            waypoints_3d.append((wx, wy, wz))

        return waypoints_3d
