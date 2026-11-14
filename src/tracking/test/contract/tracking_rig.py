"""Synchronous DDS harness; ROS time advances only at explicit fixture steps."""
import time
import pytest
rclpy = pytest.importorskip('rclpy')
from rclpy.context import Context
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.parameter import Parameter
from rosgraph_msgs.msg import Clock
from builtin_interfaces.msg import Time


def stamp(t):
    ns = round(t * 1e9)
    return Time(sec=ns // 10**9, nanosec=ns % 10**9)


def seconds(t):
    return t.sec + t.nanosec * 1e-9


class Rig:
    def __init__(self, cls, params=None):
        self.context = Context()
        rclpy.init(context=self.context)
        self.node = cls(context=self.context, parameter_overrides=[
            Parameter(k, value=v) for k, v in dict(use_sim_time=True, **(params or {})).items()])
        self.client = Node('fixture_client', context=self.context)
        self.exec = SingleThreadedExecutor(context=self.context)
        self.exec.add_node(self.node)
        self.exec.add_node(self.client)
        self.clock = self.client.create_publisher(Clock, '/clock', 10)
        self.wait(lambda: self.clock.get_subscription_count() > 0)
        self.set_time(100.)
        assert self.node.trigger_configure().name == 'SUCCESS'
        assert self.node.trigger_activate().name == 'SUCCESS'

    def wait(self, pred, timeout=8.):
        end = time.monotonic() + timeout
        while not pred() and time.monotonic() < end:
            self.exec.spin_once(timeout_sec=.01)
        assert pred(), 'DDS condition timeout'

    def spin(self, duration=.15):
        end = time.monotonic() + duration
        while time.monotonic() < end:
            self.exec.spin_once(timeout_sec=.01)

    def set_time(self, t):
        self.clock.publish(Clock(clock=stamp(t)))
        self.wait(lambda: abs(self.node.get_clock().now().nanoseconds * 1e-9 - t) < 1e-7)
        self.t = t

    def record(self, typ, topic, qos=100):
        rows = []
        self.client.create_subscription(typ, topic, rows.append, qos)
        return rows

    def publisher(self, typ, topic):
        pub = self.client.create_publisher(typ, topic, 5)
        self.wait(lambda: pub.get_subscription_count() > 0)
        return pub

    def call(self, typ, topic, **fields):
        client = self.client.create_client(typ, topic)
        assert client.wait_for_service(timeout_sec=8.)
        future = client.call_async(typ.Request(**fields))
        self.wait(future.done)
        result = future.result()
        self.client.destroy_client(client)
        return result

    def close(self):
        self.exec.shutdown()
        self.node.destroy_node()
        self.client.destroy_node()
        self.context.shutdown()
