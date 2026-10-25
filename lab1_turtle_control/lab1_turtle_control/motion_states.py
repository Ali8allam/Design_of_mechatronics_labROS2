"""goToPose motion logic, written WITHOUT any ROS imports so it can be unit-tested.

Design pattern: STATE.
The motion has three phases, each one a class with its own control law and its own
rule for when to hand over to the next phase:

    RotateToHeadingState -> DriveState -> RotateToYawState -> DoneState

`GoToPoseController` is the context: it holds the current state and delegates to it.
Adding a phase (e.g. "back up first") means adding one class, not editing an if/elif chain.
"""
from __future__ import annotations

import math
from abc import ABC, abstractmethod
from dataclasses import dataclass
from typing import Tuple


# --------------------------------------------------------------------------- helpers
def normalize_angle(angle: float) -> float:
    """Wrap an angle to [-pi, pi]."""
    return math.atan2(math.sin(angle), math.cos(angle))


def clamp(value: float, limit: float) -> float:
    """Clamp value to [-limit, +limit]."""
    return max(-limit, min(limit, value))


# --------------------------------------------------------------------------- data
@dataclass(frozen=True)
class Pose2D:
    x: float
    y: float
    theta: float


@dataclass(frozen=True)
class Goal2D:
    x: float
    y: float
    yaw: float


@dataclass(frozen=True)
class SpeedLimits:
    max_linear: float
    max_angular: float


@dataclass(frozen=True)
class ControlConfig:
    position_tolerance: float = 0.05     # stop when closer than this to the target
    angle_tolerance: float = 0.03        # final yaw tolerance (rad)
    heading_align_tolerance: float = 0.05  # start driving once aligned within this (rad)
    realign_threshold: float = 0.6       # stop and re-rotate if the heading error exceeds this
    close_range: float = 0.25            # inside this radius heading corrections are frozen
    k_linear: float = 1.5
    k_rotate: float = 4.0
    k_heading: float = 6.0
    min_linear_speed: float = 0.1        # avoids crawling the last centimetres


@dataclass(frozen=True)
class Command:
    linear: float = 0.0
    angular: float = 0.0


@dataclass(frozen=True)
class Errors:
    distance: float   # distance to the target point
    heading: float    # angle from the turtle's heading to the target point
    yaw: float        # angle from the turtle's heading to the requested final yaw


@dataclass(frozen=True)
class StepResult:
    command: Command
    phase: str
    distance_remaining: float
    angle_remaining: float
    done: bool


def compute_errors(pose: Pose2D, goal: Goal2D) -> Errors:
    dx, dy = goal.x - pose.x, goal.y - pose.y
    return Errors(
        distance=math.hypot(dx, dy),
        heading=normalize_angle(math.atan2(dy, dx) - pose.theta),
        yaw=normalize_angle(goal.yaw - pose.theta),
    )


# --------------------------------------------------------------------------- states
class MotionState(ABC):
    name = "state"
    finished = False

    @abstractmethod
    def step(self, err: Errors, limits: SpeedLimits,
             cfg: ControlConfig) -> Tuple[Command, "MotionState"]:
        """Return (velocity command, next state). Return `self` to stay in this phase."""

    def angle_remaining(self, err: Errors) -> float:
        return err.heading


class RotateToHeadingState(MotionState):
    """Turn in place until the turtle faces the target point."""
    name = "rotate_to_heading"

    def step(self, err, limits, cfg):
        if err.distance <= cfg.position_tolerance:
            return Command(), RotateToYawState()
        if abs(err.heading) <= cfg.heading_align_tolerance:
            return Command(), DriveState()
        w = clamp(cfg.k_rotate * err.heading, limits.max_angular)
        return Command(0.0, w), self


class DriveState(MotionState):
    """Drive forward, steering to stay on the line to the target."""
    name = "drive"

    def step(self, err, limits, cfg):
        if err.distance <= cfg.position_tolerance:
            return Command(), RotateToYawState()
        # Overshot (target is behind us) or drifted badly off the line -> turn again.
        if abs(err.heading) > math.pi / 2 or (
                err.distance > cfg.close_range and abs(err.heading) > cfg.realign_threshold):
            return Command(), RotateToHeadingState()

        v_floor = min(cfg.min_linear_speed, limits.max_linear)
        v = min(max(cfg.k_linear * err.distance, v_floor), limits.max_linear)
        if err.distance > cfg.close_range:
            v *= max(0.0, math.cos(err.heading))          # slow down while steering
            w = clamp(cfg.k_heading * err.heading, limits.max_angular)
        else:
            w = 0.0                                       # final approach: go straight
        return Command(v, w), self


class RotateToYawState(MotionState):
    """Turn in place to the requested final yaw."""
    name = "rotate_to_yaw"

    def step(self, err, limits, cfg):
        if abs(err.yaw) <= cfg.angle_tolerance:
            return Command(), DoneState()
        w = clamp(cfg.k_rotate * err.yaw, limits.max_angular)
        return Command(0.0, w), self

    def angle_remaining(self, err):
        return err.yaw


class DoneState(MotionState):
    name = "done"
    finished = True

    def step(self, err, limits, cfg):
        return Command(), self

    def angle_remaining(self, err):
        return err.yaw


# --------------------------------------------------------------------------- context
class GoToPoseController:
    """Call `update()` once per control tick with the latest pose and the CURRENT limits."""

    def __init__(self, goal: Goal2D, cfg: ControlConfig = ControlConfig()):
        self.goal = goal
        self.cfg = cfg
        self._state: MotionState = RotateToHeadingState()

    @property
    def phase(self) -> str:
        return self._state.name

    def update(self, pose: Pose2D, limits: SpeedLimits) -> StepResult:
        err = compute_errors(pose, self.goal)
        state = self._state
        command, next_state = state.step(err, limits, self.cfg)
        self._state = next_state
        return StepResult(
            command=command,
            phase=state.name,
            distance_remaining=err.distance,
            angle_remaining=state.angle_remaining(err),
            done=next_state.finished,
        )
