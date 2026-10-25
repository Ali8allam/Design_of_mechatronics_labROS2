"""Unit tests for the goToPose logic, using a simulated turtle (same Euler update as turtlesim)."""
import math

from lab1_turtle_control.constants import START_THETA, START_X, START_Y
from lab1_turtle_control.motion_states import (
    ControlConfig, Goal2D, GoToPoseController, Pose2D, SpeedLimits, normalize_angle)
from lab1_turtle_control.square_geometry import square_corners

DT = 0.02
START = Pose2D(START_X, START_Y, START_THETA)
FAST = SpeedLimits(2.0, 2.0)
CFG = ControlConfig()


def simulate(start, goal, limits, dt=DT, max_time=90.0):
    """Run the controller against a fake turtle. Returns (final pose, time, peak v, peak w, path)."""
    ctrl = GoToPoseController(goal, CFG)
    pose, t, peak_v, peak_w, path = start, 0.0, 0.0, 0.0, [start]
    while t < max_time:
        step = ctrl.update(pose, limits)
        if step.done:
            return pose, t, peak_v, peak_w, path
        v, w = step.command.linear, step.command.angular
        peak_v, peak_w = max(peak_v, abs(v)), max(peak_w, abs(w))
        nx = pose.x + math.cos(pose.theta) * v * dt
        ny = pose.y + math.sin(pose.theta) * v * dt
        assert 0.0 <= nx <= 11.0889 and 0.0 <= ny <= 11.0889, "turtle hit the wall"
        pose = Pose2D(nx, ny, normalize_angle(pose.theta + w * dt))
        path.append(pose)
        t += dt
    raise AssertionError(f"goal {goal} did not converge within {max_time}s")


def assert_reached(pose, goal, pos_tol=0.1, yaw_tol=0.05):
    assert math.hypot(goal.x - pose.x, goal.y - pose.y) <= pos_tol, (pose, goal)
    assert abs(normalize_angle(goal.yaw - pose.theta)) <= yaw_tol, (pose, goal)


def test_goals_in_all_directions_and_yaws():
    targets = [(8.0, 5.5), (2.0, 5.5), (5.5, 9.0), (5.5, 2.0),   # right, left, up, down
               (9.0, 9.0), (1.0, 1.0), (1.0, 10.0), (10.0, 1.0), # diagonals
               (5.5, 5.5), (5.6, 5.5), (0.5, 0.5), (10.5, 10.5)] # no-move, tiny move, near walls
    yaws = [0.0, 1.57, -1.57, 3.14, -3.14, 2.5, -0.3]
    for x, y in targets:
        for yaw in yaws:
            goal = Goal2D(x, y, yaw)
            pose, *_ = simulate(START, goal, FAST)
            assert_reached(pose, goal)


def test_start_from_arbitrary_headings():
    for theta in (-3.0, -1.5, 0.0, 1.5, 3.0, math.pi):
        start = Pose2D(3.0, 7.0, theta)
        goal = Goal2D(8.0, 2.0, 0.7)
        pose, *_ = simulate(start, goal, FAST)
        assert_reached(pose, goal)


def test_speed_limits_are_respected():
    limits = SpeedLimits(0.5, 0.8)
    pose, _, peak_v, peak_w, _ = simulate(START, Goal2D(9.0, 9.0, 2.0), limits)
    assert peak_v <= 0.5 + 1e-9 and peak_w <= 0.8 + 1e-9
    assert_reached(pose, Goal2D(9.0, 9.0, 2.0))


def test_lower_limits_are_slower():
    goal = Goal2D(9.0, 9.0, 2.0)
    _, t_fast, *_ = simulate(START, goal, SpeedLimits(2.0, 2.0))
    _, t_slow, *_ = simulate(START, goal, SpeedLimits(0.5, 0.5))
    assert t_slow > 2 * t_fast


def test_limits_can_change_while_moving():
    """Limits are passed at every tick, so a mid-motion change takes effect immediately."""
    goal = Goal2D(10.0, 5.5, 0.0)
    ctrl = GoToPoseController(goal, CFG)
    pose, t = START, 0.0
    peak_before, peak_after = 0.0, 0.0
    while True:
        limits = SpeedLimits(2.0, 2.0) if t < 1.0 else SpeedLimits(0.3, 2.0)
        step = ctrl.update(pose, limits)
        if step.done:
            break
        v, w = step.command.linear, step.command.angular
        if t < 1.0:
            peak_before = max(peak_before, v)
        elif t > 1.05:
            peak_after = max(peak_after, v)
        pose = Pose2D(pose.x + math.cos(pose.theta) * v * DT, pose.y + math.sin(pose.theta) * v * DT,
                      normalize_angle(pose.theta + w * DT))
        t += DT
        assert t < 90
    assert peak_before > 1.0 and peak_after <= 0.3 + 1e-9
    assert_reached(pose, goal)


def test_square_sequence_is_a_clean_square():
    for side in (1.0, 3.0, 5.0):
        pose = START
        corners = square_corners(side)
        assert len(corners) == 4
        for wp in corners:
            goal = Goal2D(wp.x, wp.y, wp.yaw)
            end, _, _, _, path = simulate(pose, goal, FAST)
            assert_reached(end, goal)
            # every side must be a straight line (small lateral deviation)
            ax, ay = pose.x, pose.y
            length = math.hypot(goal.x - ax, goal.y - ay)
            for p in path:
                cross = abs((goal.x - ax) * (ay - p.y) - (ax - p.x) * (goal.y - ay)) / length
                assert cross < 0.15, f"side deviates {cross:.3f} from straight line"
            pose = end
