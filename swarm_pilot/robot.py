"""Differential-drive robot with a virtual lidar and PID steering."""

from __future__ import annotations

import math
import random
from dataclasses import dataclass, field

from .pid import PID


def wrap_angle(a: float) -> float:
    while a > math.pi:
        a -= 2 * math.pi
    while a < -math.pi:
        a += 2 * math.pi
    return a


@dataclass
class Obstacle:
    x: float
    y: float
    r: float


@dataclass
class Robot:
    x: float
    y: float
    heading: float = 0.0
    speed: float = 0.0
    pid: PID = field(default_factory=lambda: PID(2.0, 0.05, 0.3))
    radius: float = 0.6
    lidar_range: float = 8.0
    rng: random.Random = field(default_factory=random.Random)

    def lidar(self, obstacles: list[Obstacle]) -> float:
        """Distance to the nearest obstacle in front, or lidar_range."""
        best = self.lidar_range
        for obs in obstacles:
            dx, dy = obs.x - self.x, obs.y - self.y
            dist = math.hypot(dx, dy) - obs.r
            # Only count obstacles roughly in front.
            angle = math.atan2(dy, dx) - self.heading
            if abs(wrap_angle(angle)) < 0.6 and dist < best:
                best = max(dist, 0.0)
        return best

    def step(self, target: tuple[float, float], obstacles: list[Obstacle],
             neighbours: list["Robot"], dt: float = 0.1) -> None:
        # --- PID steering toward the target ---
        desired = math.atan2(target[1] - self.y, target[0] - self.x)
        heading_error = wrap_angle(desired - self.heading)
        turn = self.pid.update(heading_error, dt)
        self.heading = wrap_angle(self.heading + turn * dt)

        # --- Speed: slow down near obstacles and near the target ---
        dist_target = math.hypot(target[0] - self.x, target[1] - self.y)
        obstacle_dist = self.lidar(obstacles)
        desired_speed = 3.0
        if obstacle_dist < 3.0:
            desired_speed = 1.0
            # Steer hard away from the obstacle.
            self.heading = wrap_angle(self.heading + 2.5 * dt)
        if dist_target < 5.0:
            desired_speed = min(desired_speed, dist_target * 0.5)
        self.speed += (desired_speed - self.speed) * 0.2

        # --- Flocking: keep a little distance from neighbours ---
        fx = fy = 0.0
        for other in neighbours:
            if other is self:
                continue
            dx, dy = self.x - other.x, self.y - other.y
            d = math.hypot(dx, dy)
            if 0 < d < 2.5:
                fx += dx / d
                fy += dy / d
        # --- Obstacle repulsion, so robots go around instead of through ---
        ox = oy = 0.0
        for obs in obstacles:
            dx, dy = self.x - obs.x, self.y - obs.y
            d = math.hypot(dx, dy)
            if 0 < d < obs.r + 3.0:
                ox += dx / d * (obs.r + 3.0 - d)
                oy += dy / d * (obs.r + 3.0 - d)
        # --- Move, with a little sensor noise (like a real robot) ---
        noise = self.rng.gauss(0, 0.02)
        self.x += (math.cos(self.heading) * self.speed + fx * 0.3 + ox * 0.5) * dt
        self.y += (math.sin(self.heading) * self.speed + fy * 0.3 + oy * 0.5 + noise) * dt
