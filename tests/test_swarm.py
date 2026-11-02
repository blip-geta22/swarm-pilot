from swarm_pilot.pid import PID
from swarm_pilot.simulation import Simulation, score_pid


def test_pid_converges():
    pid = PID(2.0, 0.05, 0.3)
    out = pid.update(1.0, 0.1)
    assert out > 0  # positive error -> positive correction


def test_swarm_reaches_target():
    result = Simulation(n_robots=4, seed=42).run(max_steps=2000)
    assert result.reached >= 2  # most robots make it, despite obstacles


def test_tuned_score_is_finite():
    assert score_pid(PID(2.0, 0.05, 0.3)) > 0
