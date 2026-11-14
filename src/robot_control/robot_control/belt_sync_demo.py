"""Deterministic XY plant demo; never connects to ROS or hardware."""
import numpy as np
from .belt_sync import BeltSynchronizer, PlanSample


def run():
    c = BeltSynchronizer()
    c.arm(1,7,2)
    dt = .01
    cap = np.array([.35,0.])
    tcp = np.array([.34,.005])
    actual_v = np.zeros(2)
    errors = []
    for k in range(1000):
        t = k*dt
        v = np.array([.02 if t < 5. else .03,0.])
        if k % 3 == 0:   # synthetic vision/plan about 33 Hz
            p = PlanSample(1,7,2,k+1,t,t,t+.2,t+.25,tuple(cap+v*.2),tuple(v))
            assert c.accept(p,t,t)
        out = c.step(t,t,tuple(tcp),t,'robot_base')
        if not out.active:
            raise RuntimeError(out.reason)
        if t > 1:
            errors.append(float(np.linalg.norm(cap-tcp)))
        # First-order TCP velocity response: assumed 50 ms time constant.
        actual_v += (np.array(out.velocity_xy_mps)-actual_v)*(dt/.05)
        tcp += actual_v*dt
        cap += v*dt
    p95 = float(np.percentile(errors,95))
    print(f'SYNTHETIC ONLY: XY error p95 after 1 s = {p95:.7f} m')
    print('10 s simulation; belt 0.02 -> 0.03 m/s; assumed TCP lag 0.05 s.')
    print('No hardware command, no measured field accuracy, no gripper release.')
    assert p95 < .001
    return p95


if __name__ == '__main__':
    run()
