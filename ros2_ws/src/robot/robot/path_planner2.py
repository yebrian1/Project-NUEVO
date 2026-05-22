from __future__ import annotations
import math
import numpy as np

class PathPlanner:
    def compute_velocity(self, pose: tuple[float, float, float], waypoints: list[tuple[float, float]], max_linear: float) -> tuple[float, float]:
        raise NotImplementedError
    def get_obstacles(self) -> list:
        return []

class PurePursuitPlanner(PathPlanner):
    def __init__(self, lookahead_dist: float = 150, max_angular: float = 2.0, goal_tolerance: float = 20.0) -> None:
        self._lookahead  = lookahead_dist
        self._max_angular = max_angular
        self.goal_tolerance = goal_tolerance

    def compute_velocity(self, pose: tuple[float, float, float], waypoints: list[tuple[float, float]], max_linear: float) -> tuple[float, float]:
        x, y, theta = pose
        tx, ty = self._lookahead_point(x, y, waypoints)
        return self.compute_velocity_to_point(pose, (tx, ty), max_linear)

    def compute_velocity_to_point(self, pose: tuple[float, float, float], target: tuple[float, float], max_linear: float) -> tuple[float, float]:
        x, y, theta = pose
        tx, ty = target
        dx, dy = tx - x, ty - y
        x_r = math.cos(theta) * dx + math.sin(theta) * dy
        y_r = -math.sin(theta) * dx + math.cos(theta) * dy
        dist = math.hypot(x_r, y_r)
        if dist < 1e-6: return 0.0, 0.0

        direction = 1.0
        if x_r < -100.0: direction = -1.0
        
        curvature = 2.0 * y_r / (dist * dist)
        forward_scale = abs(x_r) / dist
        curvature_scale = 1.0 + abs(curvature) * self._lookahead
        linear = max_linear * forward_scale / curvature_scale * direction

        if abs(linear) <= 1e-6:
            angular = self._max_angular * math.tanh(y_r / max(self._lookahead, 1e-6))
            return 0.0, angular

        angular = curvature * linear
        if abs(angular) > self._max_angular:
            angular = math.copysign(self._max_angular, angular)
            linear = min(abs(linear), abs(angular / curvature)) * direction if abs(curvature) > 1e-6 else linear

        return linear, angular

    def _lookahead_point(self, x: float, y: float, waypoints: list[tuple[float, float]]) -> tuple[float, float]:
        for wx, wy in waypoints:
            if math.hypot(wx - x, wy - y) >= self._lookahead:
                return wx, wy
        return waypoints[-1]
    
    def CurrentTargetReached(self, target_x, target_y, x, y):
        return np.hypot(target_x - x, target_y - y) < self.goal_tolerance

def generate_maze_waypoints(start_x: float, start_y: float, start_theta_deg: float) -> list[tuple[float, float]]:
    waypoints = []
    x, y = start_x, start_y
    theta_rad = math.radians(start_theta_deg)
    step = 50.0
    
    # 1. 2300 mm Forward
    dist = step
    while dist <= 2300.0:
        waypoints.append((x + dist * math.cos(theta_rad), y + dist * math.sin(theta_rad)))
        dist += step
    x += 2300.0 * math.cos(theta_rad)
    y += 2300.0 * math.sin(theta_rad)
    
    # 2. Right 90
    theta_rad -= math.radians(90.0)
    
    # 3. 610 mm Forward
    dist = step
    while dist <= 610.0:
        waypoints.append((x + dist * math.cos(theta_rad), y + dist * math.sin(theta_rad)))
        dist += step
    x += 610.0 * math.cos(theta_rad)
    y += 610.0 * math.sin(theta_rad)
    
    # 4. Right 90
    theta_rad -= math.radians(90.0)
    
    # 5. Tail segment (300mm)
    dist = step
    while dist <= 300.0:
        waypoints.append((x + dist * math.cos(theta_rad), y + dist * math.sin(theta_rad)))
        dist += step
        
    return waypoints
