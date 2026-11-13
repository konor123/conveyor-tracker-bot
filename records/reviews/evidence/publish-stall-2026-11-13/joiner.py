"""새 참여자: kind=sub면 stall_t 구독, other면 다른 토픽 구독. 참여자 생성 시각을 출력하고 hold_s 동안 spin."""
import sys, time
import rclpy
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from std_msgs.msg import String
kind, hold_s, tag = sys.argv[1], float(sys.argv[2]), sys.argv[3]
rclpy.init()
node = rclpy.create_node(f'joiner_{tag}')
qos = QoSProfile(depth=20, reliability=ReliabilityPolicy.RELIABLE, durability=DurabilityPolicy.VOLATILE)
node.create_subscription(String, 'stall_t' if kind == 'sub' else f'other_{tag}', lambda m: None, qos)
print(f'JOIN {tag} {kind} {time.time():.3f}', flush=True)
end = time.time() + hold_s
while time.time() < end:
    rclpy.spin_once(node, timeout_sec=0.1)
node.destroy_node(); rclpy.try_shutdown()
