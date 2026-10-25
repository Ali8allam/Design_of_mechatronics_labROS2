"""Reset service: /reset_turtle (std_srvs/srv/Trigger).

Clears the screen and teleports turtle1 back to its start pose using ONLY turtlesim's own
services:  /turtle1/set_pen (off) -> /turtle1/teleport_absolute -> /clear -> /turtle1/set_pen (on)
The pen is lifted during the teleport so no line is drawn from the old position to the start.
"""
import rclpy
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.node import Node
from std_srvs.srv import Empty, Trigger
from turtlesim.srv import SetPen, TeleportAbsolute

from .constants import DEFAULT_PEN, START_THETA, START_X, START_Y
from .ros_utils import call_sync, spin_multithreaded


class ResetService(Node):
    def __init__(self):
        super().__init__('reset_service')
        group = ReentrantCallbackGroup()
        self._clear = self.create_client(Empty, '/clear', callback_group=group)
        self._teleport = self.create_client(
            TeleportAbsolute, '/turtle1/teleport_absolute', callback_group=group)
        self._set_pen = self.create_client(SetPen, '/turtle1/set_pen', callback_group=group)
        self.create_service(Trigger, 'reset_turtle', self._on_reset, callback_group=group)
        self.get_logger().info('reset_turtle service ready')

    def _pen_request(self, off: bool) -> SetPen.Request:
        req = SetPen.Request()
        req.r, req.g, req.b, req.width = DEFAULT_PEN
        req.off = 1 if off else 0
        return req

    def _on_reset(self, _request, response):
        for client in (self._clear, self._teleport, self._set_pen):
            if not client.wait_for_service(timeout_sec=2.0):
                response.success = False
                response.message = f'Turtlesim service {client.srv_name} not available'
                self.get_logger().error(response.message)
                return response

        pen_lifted = False
        try:
            if call_sync(self._set_pen, self._pen_request(off=True)) is None:
                raise RuntimeError('set_pen (off) timed out')
            pen_lifted = True

            tp = TeleportAbsolute.Request()
            tp.x, tp.y, tp.theta = START_X, START_Y, START_THETA
            if call_sync(self._teleport, tp) is None:
                raise RuntimeError('teleport_absolute timed out')

            if call_sync(self._clear, Empty.Request()) is None:
                raise RuntimeError('clear timed out')

            response.success = True
            response.message = (f'Reset done: cleared screen, turtle at '
                                f'({START_X:.3f}, {START_Y:.3f}, {START_THETA:.1f})')
        except RuntimeError as exc:
            response.success = False
            response.message = f'Reset failed: {exc}'
        finally:
            if pen_lifted:                      # always put the pen back down
                call_sync(self._set_pen, self._pen_request(off=False))

        self.get_logger().info(response.message)
        return response


def main(args=None):
    rclpy.init(args=args)
    spin_multithreaded(ResetService())


if __name__ == '__main__':
    main()
