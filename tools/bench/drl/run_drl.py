"""DRL 실행 후 레지스터 15~23(바이트 단위) 읽기: 15 완료, 16-17 모델, 18-19 위치 하위 워드, 20-21 상위 워드, 22-23 현재 전류."""
import sys, time, rclpy
from dsr_msgs2.srv import DrlStart, GetDrlState, GetOutputRegisterInt
rclpy.init(); n = rclpy.create_node('run_drl')
def call(t, name, **kw):
    c = n.create_client(t, name); assert c.wait_for_service(5.0), name
    f = c.call_async(t.Request(**kw)); rclpy.spin_until_future_complete(n, f, timeout_sec=10); return f.result()
def reg(a):
    return (call(GetOutputRegisterInt, '/dsr01/plc/get_output_register_int', address=a, timeout_ms=1000).value >> 24) & 0xFF
before = reg(15)
ok = call(DrlStart, '/dsr01/drl/drl_start', robot_system=0, code=open(sys.argv[1]).read()).success
t0 = time.monotonic()
while time.monotonic() - t0 < float(sys.argv[2]) if len(sys.argv) > 2 else 8.0:
    time.sleep(0.2)
    if call(GetDrlState, '/dsr01/drl/get_drl_state').drl_state != 0:
        break
w = lambda a: reg(a) * 256 + reg(a + 1)
pos = w(18) + 65536 * w(20)
pos = pos - (1 << 32) if pos >= 1 << 31 else pos
cur = w(22); cur = cur - 65536 if cur >= 32768 else cur
print(f'drl_start={ok} done_flag={reg(15)} model={w(16)} present_position={pos} pulse present_current={cur} mA '
      f'drl_time={time.monotonic() - t0:.1f}s')
