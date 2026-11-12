"""ROS 2 shadow computation only. Publishes /tracking_preview/cmd_vel.

Never publishes Doosan commands, never opens a gripper. Run using python -m.
Identity must be explicitly supplied; a different plan requires a fresh invocation.
"""
import argparse
import time
from .belt_sync import BeltSynchronizer, PlanSample, SyncConfig


def seconds(t):
    return t.sec + t.nanosec*1e-9


def sample(msg):
    p,v = msg.target_tcp_pose.position,msg.target_velocity
    return PlanSample(msg.plan_id,msg.target_track_id,msg.target_epoch,msg.revision,
                      seconds(msg.header.stamp),seconds(msg.source_state_stamp),
                      seconds(msg.release_time),seconds(msg.valid_until),
                      (p.x,p.y),(v.x,v.y),msg.header.frame_id,msg.status,msg.position_std,p.z)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--plan-id',type=int,required=True)
    parser.add_argument('--track-id',type=int,required=True)
    parser.add_argument('--epoch',type=int,required=True)
    parser.add_argument('--config',help='JSON file containing SyncConfig fields')
    parser.add_argument('--csv',help='New CSV path; existing file is never overwritten')
    args, ros_args = parser.parse_known_args()
    import json
    from .sync_metrics import SyncCsv
    cfg = SyncConfig(**json.loads(open(args.config).read())) if args.config else SyncConfig()
    import rclpy
    from rclpy.node import Node
    from rclpy.clock import Clock, ClockType
    from rclpy.qos import qos_profile_sensor_data
    from geometry_msgs.msg import TwistStamped
    from std_msgs.msg import String
    from drop_interfaces.msg import InterceptPlan, RobotStatus

    class Preview(Node):
        def __init__(self):
            super().__init__('belt_sync_preview')
            self.core = BeltSynchronizer(cfg)
            self.csv = SyncCsv(args.csv) if args.csv else None
            self.core.arm(args.plan_id,args.track_id,args.epoch)
            self.tcp = None
            self.tcp_received = None
            self.previous_ros = None
            self.pub = self.create_publisher(TwistStamped,'/tracking_preview/cmd_vel',10)
            self.diag = self.create_publisher(String,'/tracking_preview/status',10)
            self.create_subscription(InterceptPlan,'/drop/intercept_plan',self.on_plan,qos_profile_sensor_data)
            self.create_subscription(RobotStatus,'/robot/status',self.on_tcp,qos_profile_sensor_data)
            self.timer = self.create_timer(.01,self.tick,clock=Clock(clock_type=ClockType.STEADY_TIME))
            self.last_reason = None
            self.get_logger().info('SHADOW ONLY: zero Z/angular speed; no robot output')

        def now_s(self):
            return self.get_clock().now().nanoseconds*1e-9

        def on_plan(self,msg):
            self.core.accept(sample(msg),self.now_s(),time.monotonic())

        def on_tcp(self,msg):
            if self.tcp is not None and seconds(msg.header.stamp) <= seconds(self.tcp.header.stamp):
                return  # duplicates must not refresh the receipt watchdog
            self.tcp,self.tcp_received = msg,time.monotonic()

        def tick(self):
            now,steady = self.now_s(),time.monotonic()
            if self.previous_ros is not None and now < self.previous_ros:
                self.core.stop('CLOCK_BACKWARD')
            self.previous_ros = now
            if self.tcp is None:
                self.emit((0.,0.),'WAIT_TCP')
                return
            m = self.tcp
            if not m.connected or not m.ready or m.protective_stop:
                self.core.stop('ROBOT_NOT_READY')
            if steady-self.tcp_received > self.core.cfg.max_tcp_age_s:
                self.core.stop('TCP_RECEIVE_TIMEOUT')
            out = self.core.step(now,steady,(m.tcp_pose.position.x,m.tcp_pose.position.y),
                                 seconds(m.header.stamp),m.header.frame_id)
            if self.csv:
                self.csv.write(self.core,out,now,steady,(m.tcp_pose.position.x,m.tcp_pose.position.y),seconds(m.header.stamp))
            self.emit(out.velocity_xy_mps,out.reason)

        def emit(self,xy,reason):
            msg = TwistStamped()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = 'robot_base'
            msg.twist.linear.x,msg.twist.linear.y = map(float,xy)
            self.pub.publish(msg)
            if reason != self.last_reason:
                status = String()
                status.data = reason
                self.diag.publish(status)
                self.get_logger().info(reason)
                self.last_reason = reason

    rclpy.init(args=ros_args)
    node = Preview()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node.csv:
            node.csv.close()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
