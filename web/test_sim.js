// Headless test for the exact browser physics in sim.js.
// Run: node web/test_sim.js
const { runSwarm } = require("./sim.js");
const assert = require("assert");

for (const kp of [0.5, 2.0, 5.0]) {
  const r = runSwarm({ kp, ki: 0.05, kd: 0.3 });
  console.log(`Kp=${kp}: ${r.reached}/8 reached in ${r.steps} steps, ` +
    `${r.collisions} collisions, avg path ${r.avgPath.toFixed(0)}px, ` +
    `avg heading change ${r.avgTurn.toFixed(1)} rad`);
  assert(r.reached === 8, `Kp=${kp} should reach`);
  assert(r.collisions < 50, `Kp=${kp} collisions should be far below the old 300-500, got ${r.collisions}`);
}
const slow = runSwarm({ kp: 0.5, ki: 0.05, kd: 0.3 });
const fast = runSwarm({ kp: 5.0, ki: 0.05, kd: 0.3 });
console.log(`PID effect: Kp 0.5 takes ${slow.steps} steps, Kp 5.0 takes ${fast.steps} steps, ` +
  `turning ${slow.avgTurn.toFixed(1)} vs ${fast.avgTurn.toFixed(1)} rad`);
assert(slow.steps !== fast.steps || slow.avgTurn !== fast.avgTurn,
  "PID gains should change the trajectory, not just the label");
console.log("All web-sim checks passed.");
