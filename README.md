# 🤖 SwarmPilot — Self-Tuning Robot Swarm Simulator

A swarm of autonomous robots that navigates around obstacles to a target,
steered by PID control, kept apart by flocking rules, and — the crazy part —
**it tunes its own PID gains with a genetic algorithm.**

No hardware needed. The whole thing runs in Python, and there's a live
browser demo too.

## Watch it live

Open `web/index.html` in any browser. Drag the Kp / Ki / Kd sliders and
watch the swarm react in real time. Pause, reset, and try to break it.

## Run it

```bash
# 8 robots navigate to the target
python -m swarm_pilot.cli run --robots 8

# Evolve better PID gains automatically
python -m swarm_pilot.cli tune --generations 8
```

## How it works

- **Robot:** differential-drive model, virtual lidar, sensor noise
- **Control:** PID on heading error, with anti-windup and speed control
  (slow near obstacles, slow near the target)
- **Swarm:** separation flocking so robots don't clump or collide
- **Auto-tuner:** genetic algorithm — keep the best gains, mutate and
  cross-breed them, repeat. Score = distance left + collisions × 20 + time.

## Layout

```
swarm_pilot/
  pid.py          # PID controller
  robot.py        # robot + lidar + flocking
  simulation.py   # swarm, obstacles, scoring
  tuner.py        # genetic PID auto-tuner
  cli.py          # run / tune commands
web/index.html    # live browser demo
tests/            # pytest tests
```

## Tests

```bash
python -m pytest
```
