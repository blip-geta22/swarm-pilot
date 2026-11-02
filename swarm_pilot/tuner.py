"""Genetic auto-tuner: evolve PID gains instead of hand-tuning them.

Each generation: keep the best gains, mutate them, cross-breed them.
After a few generations the swarm learns gains that beat the defaults.
"""

from __future__ import annotations

import random

from .pid import PID
from .simulation import score_pid


def evolve(generations: int = 8, population: int = 12, seed: int = 1) -> tuple[PID, float]:
    rng = random.Random(seed)

    def random_pid() -> PID:
        return PID(kp=rng.uniform(0.5, 5.0), ki=rng.uniform(0, 0.3),
                   kd=rng.uniform(0, 1.0))

    pop = [random_pid() for _ in range(population)]
    best, best_score = pop[0], float("inf")
    for gen in range(generations):
        scored = sorted((score_pid(p), p) for p in pop)
        if scored[0][0] < best_score:
            best_score, best = scored[0][0], scored[0][1]
        print(f"  gen {gen + 1}: best score {scored[0][0]:.1f} "
              f"Kp={scored[0][1].kp:.2f} Ki={scored[0][1].ki:.3f} Kd={scored[0][1].kd:.2f}")
        survivors = [p for _, p in scored[: population // 2]]
        children = []
        while len(children) < population - len(survivors):
            a, b = rng.choice(survivors), rng.choice(survivors)
            child = PID(
                kp=(a.kp + b.kp) / 2 + rng.gauss(0, 0.3),
                ki=max(0, (a.ki + b.ki) / 2 + rng.gauss(0, 0.03)),
                kd=max(0, (a.kd + b.kd) / 2 + rng.gauss(0, 0.1)),
            )
            children.append(child)
        pop = survivors + children
    return best, best_score
