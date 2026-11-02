"""Run a whole swarm to a target, score it, and report collisions."""

from __future__ import annotations

import math
import random
from dataclasses import dataclass

from .pid import PID
from .robot import Obstacle, Robot


@dataclass
class SimulationResult:
    steps: int
    reached: int
    collisions: int
    total_distance_error: float
    success: bool


class Simulation:
    def __init__(self, n_robots: int = 8, target=(40, 30), seed: int = 42,
                 pid: PID | None = None):
        self.rng = random.Random(seed)
        self.target = target
        self.robots = [
            Robot(x=self.rng.uniform(0, 8), y=self.rng.uniform(0, 40),
                  heading=0.0, pid=pid or PID(2.0, 0.05, 0.3),
                  rng=random.Random(seed + i))
            for i in range(n_robots)
        ]
        # Fresh PID state per robot (a shared PID object would share integrals).
        self.robots = [
            Robot(x=r.x, y=r.y, heading=r.heading,
                  pid=PID(pid.kp, pid.ki, pid.kd) if pid else PID(2.0, 0.05, 0.3),
                  rng=r.rng)
            for r in self.robots
        ]
        self.obstacles = [
            Obstacle(20, 15, 3), Obstacle(25, 35, 2.5), Obstacle(32, 20, 2),
        ]
        self.collisions = 0
        self.steps = 0

    def step(self, dt: float = 0.1):
        for robot in self.robots:
            robot.step(self.target, self.obstacles, self.robots, dt)
        # No driving through: push out, point away, count distinct contacts.
        # (Old code counted every frame inside an obstacle, inflating the total.)
        for robot in self.robots:
            for obs in self.obstacles:
                dx, dy = robot.x - obs.x, robot.y - obs.y
                d = math.hypot(dx, dy)
                min_d = obs.r + robot.radius
                if d < min_d:
                    self.collisions += 1
                    nx, ny = (dx / d, dy / d) if d else (1.0, 0.0)
                    robot.x, robot.y = obs.x + nx * (min_d + 0.1), obs.y + ny * (min_d + 0.1)
                    robot.heading = math.atan2(ny, nx)
                    robot.speed *= 0.3
        self.steps += 1

    def run(self, max_steps: int = 1000) -> SimulationResult:
        for _ in range(max_steps):
            self.step()
            reached = sum(
                1 for r in self.robots
                if math.hypot(r.x - self.target[0], r.y - self.target[1]) < 3.0
            )
            if reached == len(self.robots):
                break
        reached = sum(
            1 for r in self.robots
            if math.hypot(r.x - self.target[0], r.y - self.target[1]) < 3.0
        )
        error = sum(
            math.hypot(r.x - self.target[0], r.y - self.target[1])
            for r in self.robots
        )
        return SimulationResult(
            steps=self.steps, reached=reached, collisions=self.collisions,
            total_distance_error=error, success=reached == len(self.robots),
        )


def score_pid(pid: PID, seed: int = 42) -> float:
    """Lower is better: distance left + collisions + time taken."""
    sim = Simulation(pid=pid, seed=seed)
    result = sim.run()
    return result.total_distance_error + result.collisions * 20 + result.steps * 0.1
