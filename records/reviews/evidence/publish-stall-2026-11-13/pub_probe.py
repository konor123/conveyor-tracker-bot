"""publish() 소요시간 측정 발행자. 100 Hz로 reliable 토픽에 보내고 느린 호출(>= 20 ms)을 기록한다."""
import json, sys, time
import rclpy
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from std_msgs.msg import String

dur_s = float(sys.argv[1]); out = sys.argv[2]
rclpy.init()
t_init = time.time()
node = rclpy.create_node('stall_pub')
qos = QoSProfile(depth=20, reliability=ReliabilityPolicy.RELIABLE, durability=DurabilityPolicy.VOLATILE)
pub = node.create_publisher(String, 'stall_t', qos)
msg = String(data='x' * 64)
slow, n, worst = [], 0, 0.0
t0 = time.time(); next_t = time.perf_counter()
print(f'PUB_READY {t0:.3f}', flush=True)
while time.time() - t0 < dur_s:
    a = time.perf_counter(); pub.publish(msg); d = time.perf_counter() - a
    n += 1; worst = max(worst, d)
    if d >= 0.02:
        slow.append((round(time.time() - t0, 3), round(d, 3)))
    next_t += 0.01
    s = next_t - time.perf_counter()
    if s > 0: time.sleep(s)
    else: next_t = time.perf_counter()
json.dump({'t0': t0, 'init_to_ready_s': round(t0 - t_init, 3), 'n': n, 'worst_s': round(worst, 4), 'slow': slow},
          open(out, 'w'))
node.destroy_node(); rclpy.try_shutdown()
