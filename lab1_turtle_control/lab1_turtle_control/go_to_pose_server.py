"""goToPose action server.

* Action  /go_to_pose        (lab1_interfaces/action/GoToPose)  - drive turtle1 to (x, y, yaw)
* Service /set_speed_limits  (lab1_interfaces/srv/SetSpeedLimits) - change both limits live
* Params  max_linear_speed, max_angular_speed (+ tolerances and gains)

The speed limits are read from the parameters on EVERY control tick, so a change made with
`ros2 param set` or with the service takes effect while the turtle is moving.
"""
import math
import threading
import time

import rclpy
from geometry_msgs.msg import Twist
from lab1_interfaces.action import GoToPose
from lab1_interfaces.srv import SetSpeedLimits
from rcl_interfaces.msg import ParameterDescriptor, SetParametersResult
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.node import Node
from rclpy.parameter import Parameter
from turtlesim.msg import Pose

from .constants import WORLD_MAX, WORLD_MIN
from .motion_states import (Command, ControlConfig, Goal2D, GoToPoseController, Pose2D,
                            SpeedLimits, compute_errors, normalize_angle)
from .ros_utils import spin_multithreaded

SPEED_PARAMS = ('max_linear_speed', 'max_angular_speed')
POSITIVE_PARAMS = SPEED_PARAMS + ('position_tolerance', 'angle_tolerance', 'k_linear',
                                  'k_rotate', 'k_heading', 'control_rate_hz', 'goal_timeout_sec')


class GoToPoseServer(Node):
    def __init__(self):
        super().__init__('go_to_pose_server')
        self._group = ReentrantCallbackGroup()
        self._lock = threading.Lock()
        self._pose = None
        self._busy = False
        self._goal_count = 0

        # Register the validator BEFORE declaring, so bad launch/YAML values are rejected too.
        self.add_on_set_parameters_callback(self._on_set_parameters)
        self.declare_parameter('max_linear_speed', 2.0, ParameterDescriptor(
            description='Maximum linear speed [units/s], must be > 0'))
        self.declare_parameter('max_angular_speed', 2.0, ParameterDescriptor(
            description='Maximum angular speed [rad/s], must be > 0'))
        self.declare_parameter('position_tolerance', 0.05)
        self.declare_parameter('angle_tolerance', 0.03)
        self.declare_parameter('k_linear', 1.5)
        self.declare_parameter('k_rotate', 4.0)
        self.declare_parameter('k_heading', 6.0)
        self.declare_parameter('control_rate_hz', 50.0)
        self.declare_parameter('goal_timeout_sec', 60.0)

        self._cmd_pub = self.create_publisher(Twist, '/turtle1/cmd_vel', 10)
        self.create_subscription(Pose, '/turtle1/pose', self._on_pose, 10,
                                 callback_group=self._group)
        self.create_service(SetSpeedLimits, 'set_speed_limits', self._on_set_speed_limits,
                            callback_group=self._group)
        self._server = ActionServer(
            self, GoToPose, 'go_to_pose',
            execute_callback=self._execute,
            goal_callback=self._on_goal,
            cancel_callback=self._on_cancel,
            callback_group=self._group)

        self.get_logger().info(
            f'go_to_pose ready (max_linear={self._limits().max_linear:.2f}, '
            f'max_angular={self._limits().max_angular:.2f})')

    # ------------------------------------------------------------------ parameters
    def _on_set_parameters(self, params):
        for p in params:
            if p.name in POSITIVE_PARAMS:
                ok = (isinstance(p.value, (int, float)) and not isinstance(p.value, bool)
                      and math.isfinite(p.value) and p.value > 0.0)
                if not ok:
                    return SetParametersResult(
                        successful=False, reason=f'{p.name} must be a finite number > 0')
        return SetParametersResult(successful=True)

    def _limits(self) -> SpeedLimits:
        return SpeedLimits(
            max_linear=float(self.get_parameter('max_linear_speed').value),
            max_angular=float(self.get_parameter('max_angular_speed').value))

    def _control_config(self) -> ControlConfig:
        g = lambda name: float(self.get_parameter(name).value)  # noqa: E731
        return ControlConfig(
            position_tolerance=g('position_tolerance'), angle_tolerance=g('angle_tolerance'),
            k_linear=g('k_linear'), k_rotate=g('k_rotate'), k_heading=g('k_heading'))

    def _on_set_speed_limits(self, request, response):
        lin, ang = request.max_linear_speed, request.max_angular_speed
        if not (math.isfinite(lin) and math.isfinite(ang) and lin > 0.0 and ang > 0.0):
            response.success = False
            response.message = (f'Rejected: both limits must be finite and > 0 '
                                f'(got linear={lin}, angular={ang}). Limits unchanged.')
            self.get_logger().warn(response.message)
            return response
        result = self.set_parameters_atomically([
            Parameter('max_linear_speed', Parameter.Type.DOUBLE, float(lin)),
            Parameter('max_angular_speed', Parameter.Type.DOUBLE, float(ang))])
        response.success = bool(result.successful)
        response.message = (f'Speed limits set: linear={lin:.3f}, angular={ang:.3f}'
                            if result.successful else f'Failed: {result.reason}')
        self.get_logger().info(response.message)
        return response

    # ------------------------------------------------------------------ pose / cmd
    def _on_pose(self, msg: Pose):
        with self._lock:
            self._pose = Pose2D(msg.x, msg.y, msg.theta)

    def _get_pose(self):
        with self._lock:
            return self._pose

    def _publish_cmd(self, cmd: Command):
        twist = Twist()
        twist.linear.x = float(cmd.linear)
        twist.angular.z = float(cmd.angular)
        self._cmd_pub.publish(twist)

    # ------------------------------------------------------------------ action
    def _on_goal(self, request):
        t = request.target
        values = (t.x, t.y, t.yaw)
        if not all(math.isfinite(v) for v in values) or not (
                WORLD_MIN <= t.x <= WORLD_MAX and WORLD_MIN <= t.y <= WORLD_MAX):
            self.get_logger().warn(
                f'Goal rejected: ({t.x:.2f}, {t.y:.2f}) is outside the window '
                f'[{WORLD_MIN}, {WORLD_MAX}]')
            return GoalResponse.REJECT
        with self._lock:
            if self._busy:
                self.get_logger().warn('Goal rejected: another goal is still running')
                return GoalResponse.REJECT
            self._busy = True
            self._goal_count += 1
            n = self._goal_count
        self.get_logger().info(
            f'Goal #{n} accepted: x={t.x:.2f} y={t.y:.2f} yaw={t.yaw:.2f}')
        return GoalResponse.ACCEPT

    def _on_cancel(self, _goal_handle):
        self.get_logger().info('Cancel requested')
        return CancelResponse.ACCEPT

    def _execute(self, goal_handle):
        try:
            return self._run_goal(goal_handle)
        finally:
            self._publish_cmd(Command())          # always leave the turtle stopped
            with self._lock:
                self._busy = False

    def _finish(self, goal_handle, goal, success, message, status):
        result = GoToPose.Result()
        pose = self._get_pose()
        if pose is not None:
            err = compute_errors(pose, goal)
            result.distance_error = float(err.distance)
            result.yaw_error = float(err.yaw)
        result.success = success
        result.message = message
        getattr(goal_handle, status)()
        self.get_logger().info(message)
        return result

    def _run_goal(self, goal_handle):
        t = goal_handle.request.target
        goal = Goal2D(float(t.x), float(t.y), normalize_angle(float(t.yaw)))
        controller = GoToPoseController(goal, self._control_config())
        period = 1.0 / float(self.get_parameter('control_rate_hz').value)
        timeout = float(self.get_parameter('goal_timeout_sec').value)

        # Wait (briefly) for the first pose message.
        waited = 0.0
        while self._get_pose() is None and waited < 3.0 and rclpy.ok():
            time.sleep(0.05)
            waited += 0.05
        if self._get_pose() is None:
            return self._finish(goal_handle, goal, False,
                                'No pose received - is turtlesim running?', 'abort')

        start = time.monotonic()
        feedback = GoToPose.Feedback()
        while rclpy.ok():
            if goal_handle.is_cancel_requested:
                return self._finish(goal_handle, goal, False, 'Goal canceled', 'canceled')
            if time.monotonic() - start > timeout:
                return self._finish(goal_handle, goal, False,
                                    f'Timed out after {timeout:.0f}s', 'abort')

            # Limits are re-read from the parameters on every tick -> live changes.
            step = controller.update(self._get_pose(), self._limits())
            if step.done:
                break
            self._publish_cmd(step.command)

            feedback.distance_remaining = float(step.distance_remaining)
            feedback.angle_remaining = float(step.angle_remaining)
            feedback.phase = step.phase
            goal_handle.publish_feedback(feedback)
            time.sleep(period)
        else:
            return self._finish(goal_handle, goal, False, 'Shutting down', 'abort')

        return self._finish(goal_handle, goal, True, 'Goal reached', 'succeed')


def main(args=None):
    rclpy.init(args=args)
    spin_multithreaded(GoToPoseServer())


if __name__ == '__main__':
    main()
