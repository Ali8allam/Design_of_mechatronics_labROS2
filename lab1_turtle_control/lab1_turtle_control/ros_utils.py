"""Small helpers shared by the nodes."""
import threading

import rclpy
from rclpy.executors import MultiThreadedExecutor

try:  # rclpy >= Iron raises this on Ctrl-C / external shutdown
    from rclpy.executors import ExternalShutdownException
except ImportError:  # Humble
    class ExternalShutdownException(Exception):
        pass


def future_event(future) -> threading.Event:
    """Return an Event that is set when `future` completes (works if it is already done)."""
    event = threading.Event()
    future.add_done_callback(lambda _f: event.set())
    return event


def call_sync(client, request, timeout: float = 5.0):
    """Call a service from INSIDE a callback without blocking the executor.

    Needs a MultiThreadedExecutor and a ReentrantCallbackGroup on the client so another
    thread can deliver the response. Returns the response, or None on timeout.
    """
    future = client.call_async(request)
    if not future_event(future).wait(timeout):
        return None
    return future.result()


def spin_multithreaded(node, num_threads: int = 4) -> None:
    """Spin `node` with a multi-threaded executor and shut down cleanly."""
    executor = MultiThreadedExecutor(num_threads=num_threads)
    executor.add_node(node)
    try:
        executor.spin()
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
