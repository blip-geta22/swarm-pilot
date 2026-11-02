"""SwarmPilot CLI.

    python -m swarm_pilot.cli run --robots 8
    python -m swarm_pilot.cli tune --generations 8
"""

from __future__ import annotations

import argparse
import sys

from .pid import PID
from .simulation import Simulation, score_pid
from .tuner import evolve


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description="SwarmPilot robot swarm simulator")
    sub = parser.add_subparsers(required=True)

    p = sub.add_parser("run")
    p.add_argument("--robots", type=int, default=8)
    p.add_argument("--kp", type=float, default=2.0)
    p.add_argument("--ki", type=float, default=0.05)
    p.add_argument("--kd", type=float, default=0.3)
    p.set_defaults(cmd="run")

    p = sub.add_parser("tune")
    p.add_argument("--generations", type=int, default=8)
    p.set_defaults(cmd="tune")

    args = parser.parse_args(argv)
    if args.cmd == "run":
        pid = PID(args.kp, args.ki, args.kd)
        result = Simulation(n_robots=args.robots, pid=pid).run()
        print(f"{result.reached}/{args.robots} robots reached the target "
              f"in {result.steps} steps, {result.collisions} collisions, "
              f"score={score_pid(pid):.1f}")
        return 0
    print("Evolving PID gains...")
    best, score = evolve(generations=args.generations)
    print(f"Best: Kp={best.kp:.2f} Ki={best.ki:.3f} Kd={best.kd:.2f} score={score:.1f}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
