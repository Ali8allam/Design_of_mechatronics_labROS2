"""Square action server: /draw_square (lab1_interfaces/action/DrawSquare).

1. calls /reset_turtle (clean screen, known start pose)
2. sends exactly 4 goals to /go_to_pose, one per corner, waiting for each to finish
"""
import math
import threading
import time

import rclpy
from action_msgs.msg import GoalStatus
from lab1_interfaces.action import DrawSquare, GoToPose
from lab1_interfaces.msg import TargetPose
from rclpy.action import ActionClient, ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.node import Node
from std_srvs.srv import Trigger

from .ros_utils import call_sync, future_event, spin_multithreaded
from .square_geometry import square_corners


class SquareActionServer(Node):
    def __init__(self):
        super().__init__('square_action_server')
        group = ReentrantCallbackGroup()
        self._lock = threading.Lock()
        self._busy = False

        self.declare_parameter('default_side_length', 3.0)
        self.declare_parameter('max_side_length', 5.0)
        self.declare_parameter('settle_time_sec', 0.3)

        self._reset_client = self.create_client(Trigger, 'reset_turtle', callback_group=group)
        self._goto_client = ActionClient(self, GoToPose, 'go_to_pose', callback_group=group)
        self._server = ActionServer(
            self, DrawSquare, 'draw_square',
            execute_callback=self._execute,
            goal_callback=self._on_goal,
            cancel_callback=lambda _gh: CancelResponse.ACCEPT,
            callback_group=group)
        self.get_logger().info('draw_square ready')

    # ------------------------------------------------------------------ goal handling
    def _on_goal(self, request):
        side = request.side_length
        max_side = float(self.get_parameter('max_side_length').value)
        if not math.isfinite(side) or side < 0.0 or side > max_side:
            self.get_logger().warn(
                f'Goal rejected: side_length must be in [0, {max_side}] (0 = default)')
            return GoalResponse.REJECT
        with self._lock:
            if self._busy:
                self.get_logger().warn('Goal rejected: a square is already being drawn')
                return GoalResponse.REJECT
            self._busy = True
        return GoalResponse.ACCEPT

    def _execute(self, goal_handle):
        try:
            return self._run(goal_handle)
        finally:
            with self._lock:
                self._busy = False

    def _end(self, goal_handle, success, completed, message, status):
        result = DrawSquare.Result()
        result.success = success
        result.goals_completed = completed
        result.message = message
        getattr(goal_handle, status)()
        self.get_logger().info(message)
        return result

    # ------------------------------------------------------------------ main sequence
    def _run(self, goal_handle):
        side = float(goal_handle.request.side_length)
        if side == 0.0:
            side = float(self.get_parameter('default_side_length').value)
        self.get_logger().info(f'Drawing a square with side {side:.2f}')

        fb = DrawSquare.Feedback()
        fb.current_side = 0
        fb.status = 'resetting'
        goal_handle.publish_feedback(fb)

        # --- 1. reset: clean screen + known start pose
        if not self._reset_client.wait_for_service(timeout_sec=3.0):
            return self._end(goal_handle, False, 0, 'reset_turtle service not available', 'abort')
        reset = call_sync(self._reset_client, Trigger.Request(), timeout=10.0)
        if reset is None or not reset.success:
            msg = 'Reset failed' if reset is None else f'Reset failed: {reset.message}'
            return self._end(goal_handle, False, 0, msg, 'abort')
        time.sleep(float(self.get_parameter('settle_time_sec').value))  # let pose topic update

        if not self._goto_client.wait_for_server(timeout_sec=5.0):
            return self._end(goal_handle, False, 0, 'go_to_pose server not available', 'abort')

        # --- 2. four goToPose goals, strictly one after the other
        completed = 0
        for side_no, corner in enumerate(square_corners(side), start=1):
            if goal_handle.is_cancel_requested:
                return self._end(goal_handle, False, completed, 'Square canceled', 'canceled')
            outcome, message = self._go_to_corner(goal_handle, side_no, corner)
            if outcome == 'ok':
                completed += 1
            elif outcome == 'canceled':
                return self._end(goal_handle, False, completed, 'Square canceled', 'canceled')
            else:
                return self._end(goal_handle, False, completed,
                                 f'Side {side_no} failed: {message}', 'abort')

        return self._end(goal_handle, True, completed, 'Square complete (4 sides)', 'succeed')

    def _go_to_corner(self, square_handle, side_no, corner):
        goal = GoToPose.Goal()
        goal.target = TargetPose(x=float(corner.x), y=float(corner.y), yaw=float(corner.yaw))

        def on_feedback(msg):
            f = DrawSquare.Feedback()
            f.current_side = side_no
            f.status = f'side {side_no}/4: {msg.feedback.phase}'
            f.distance_remaining = msg.feedback.distance_remaining
            square_handle.publish_feedback(f)

        send_future = self._goto_client.send_goal_async(goal, feedback_callback=on_feedback)
        if not future_event(send_future).wait(5.0):
            return 'failed', 'timed out sending goal'
        handle = send_future.result()
        if not handle.accepted:
            return 'failed', 'goToPose rejected the goal'

        result_future = handle.get_result_async()
        done = future_event(result_future)
        cancel_sent = False
        while rclpy.ok() and not done.wait(0.05):
            if square_handle.is_cancel_requested and not cancel_sent:
                handle.cancel_goal_async()
                cancel_sent = True
        if not done.is_set():
            return 'failed', 'shutting down'

        wrapped = result_future.result()
        if cancel_sent:
            return 'canceled', ''
        if wrapped.status == GoalStatus.STATUS_SUCCEEDED and wrapped.result.success:
            return 'ok', ''
        return 'failed', wrapped.result.message


def main(args=None):
    rclpy.init(args=args)
    spin_multithreaded(SquareActionServer())


if __name__ == '__main__':
    main()
