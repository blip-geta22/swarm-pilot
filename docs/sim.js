// SwarmPilot simulation core - shared by the browser demo and the Node test.
// No DOM in here, so `node web/test_sim.js` can run the exact same physics
// a recruiter sees in the browser. That is the point: the demo is not a fake.

function wrap(a) {
  while (a > Math.PI) a -= 2 * Math.PI;
  while (a < -Math.PI) a += 2 * Math.PI;
  return a;
}

function makePID(kp, ki, kd) {
  return {
    kp, ki, kd, integral: 0, prev: null,
    update(error, dt) {
      // Integral clamp = anti-windup: without it a long, stuck error would
      // pile up integral and the robot would overshoot badly once freed.
      this.integral = Math.max(-10, Math.min(10, this.integral + error * dt));
      const d = this.prev === null ? 0 : (error - this.prev) / dt;
      this.prev = error;
      return Math.max(-2.5, Math.min(2.5, this.kp * error + this.ki * this.integral + this.kd * d));
    },
  };
}

const OBSTACLES = [
  { x: 400, y: 260, r: 60 },
  { x: 500, y: 420, r: 50 },
  { x: 640, y: 260, r: 40 },
];
const TARGET = { x: 900, y: 260 };
const ROBOT_R = 12;

function createSwarm(n, gains, seed) {
  // Deterministic-ish spawn so tests are comparable; browser adds jitter.
  let s = seed || 42;
  const rand = () => { s = (s * 1664525 + 1013904223) % 4294967296; return s / 4294967296; };
  const robots = [];
  for (let i = 0; i < n; i++) {
    robots.push({
      x: 20 + rand() * 80, y: 40 + rand() * 440,
      heading: 0, angularVel: 0, speed: 0,
      pid: makePID(gains.kp, gains.ki, gains.kd),
      pathLength: 0, headingChange: 0,
    });
  }
  return { robots, steps: 0, collisions: 0 };
}

function stepSwarm(state, dt) {
  state.steps++;
  for (const r of state.robots) {
    const desired = Math.atan2(TARGET.y - r.y, TARGET.x - r.x);
    // Sensor noise, like a real robot: the measured heading error is noisy,
    // so the controller cannot rely on a perfect measurement.
    const error = wrap(desired - r.heading) + (Math.random() - 0.5) * 0.04;
    // PID output is a *desired turn rate*, not a heading jump.
    // With angular inertia below, low Kp turns sluggishly and high Kp
    // overshoots - the sliders now actually change the behaviour.
    const turnRate = r.pid.update(error, dt);
    r.angularVel += (turnRate - r.angularVel) * 0.35;
    let headingBefore = r.heading;
    r.heading = wrap(r.heading + r.angularVel * dt);

    // --- Real obstacle avoidance: steer to the clear side ---
    let nearest = null, nearestDist = Infinity;
    for (const o of OBSTACLES) {
      const d = Math.hypot(o.x - r.x, o.y - r.y) - o.r;
      const ang = Math.abs(wrap(Math.atan2(o.y - r.y, o.x - r.x) - r.heading));
      if (ang < 0.9 && d < nearestDist) { nearest = o; nearestDist = d; }
    }
    let wantSpeed = 60; // px/s
    if (nearest && nearestDist < 130) {
      // Which side is the obstacle on? Turn the other way, proportionally
      // harder the closer it is. This is the fix for "nudge the same way".
      const side = Math.sign(wrap(Math.atan2(nearest.y - r.y, nearest.x - r.x) - r.heading)) || 1;
      const strength = (1 - nearestDist / 130);
      r.heading = wrap(r.heading - side * strength * 2.2 * dt);
      wantSpeed = 30;
    }
    const distT = Math.hypot(TARGET.x - r.x, TARGET.y - r.y);
    if (distT < 100) wantSpeed = Math.min(wantSpeed, distT * 0.6);
    r.speed += (wantSpeed - r.speed) * 0.15;

    // Flocking separation
    let fx = 0, fy = 0;
    for (const o of state.robots) {
      if (o === r) continue;
      const dx = r.x - o.x, dy = r.y - o.y, d = Math.hypot(dx, dy);
      if (d > 0 && d < 50) { fx += dx / d; fy += dy / d; }
    }
    const oldX = r.x, oldY = r.y;
    r.x += (Math.cos(r.heading) * r.speed + fx * 20) * dt;
    r.y += (Math.sin(r.heading) * r.speed + fy * 20) * dt;
    r.pathLength += Math.hypot(r.x - oldX, r.y - oldY);
    r.headingChange += Math.abs(wrap(r.heading - headingBefore));

    // --- No driving through: push out and count it, once per contact ---
    if (r.cooldown > 0) r.cooldown--;
    for (const o of OBSTACLES) {
      const d = Math.hypot(r.x - o.x, r.y - o.y);
      const minD = o.r + ROBOT_R;
      if (d < minD) {
        if (r.cooldown === 0) { state.collisions++; r.cooldown = 30; }
        // Push out to the surface, so a collision is a bounce, not a tunnel,
        // and point away from the obstacle so PID doesn't pull it straight back in.
        const nx = (r.x - o.x) / (d || 1), ny = (r.y - o.y) / (d || 1);
        r.x = o.x + nx * (minD + 2); r.y = o.y + ny * (minD + 2);
        r.heading = Math.atan2(ny, nx);
        r.angularVel = 0;
        r.speed *= 0.3;
      }
    }
  }
}

function runSwarm(gains, maxSteps) {
  const state = createSwarm(8, gains, 42);
  const dt = 0.1;
  while (state.steps < (maxSteps || 3000)) {
    stepSwarm(state, dt);
    const reached = state.robots.filter(
      (r) => Math.hypot(r.x - TARGET.x, r.y - TARGET.y) < 60).length;
    if (reached === 8) break;
  }
  const reached = state.robots.filter(
    (r) => Math.hypot(r.x - TARGET.x, r.y - TARGET.y) < 60).length;
  const avgPath = state.robots.reduce((s, r) => s + r.pathLength, 0) / 8;
  const avgTurn = state.robots.reduce((s, r) => s + r.headingChange, 0) / 8;
  return { steps: state.steps, reached, collisions: state.collisions, avgPath, avgTurn };
}

if (typeof module !== "undefined") {
  module.exports = { wrap, makePID, createSwarm, stepSwarm, runSwarm, OBSTACLES, TARGET };
}
