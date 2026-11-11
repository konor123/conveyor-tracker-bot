"""Offline preview telemetry. Does not send robot commands or authorize release."""
import csv
import json
import math
from pathlib import Path
import numpy as np

FIELDS = ('ros_s','steady_s','active','reason','plan_id','revision','track_id','epoch',
          'tcp_stamp_s','tcp_x_m','tcp_y_m','target_x_m','target_y_m',
          'error_xy_m','cmd_vx_mps','cmd_vy_mps','position_std_m','release_uncertainty_ok')

class SyncCsv:
    def __init__(self,path):
        # Exclusive creation: never silently overwrite an earlier measurement.
        self.file = Path(path).open('x',newline='',encoding='utf-8')
        self.writer = csv.DictWriter(self.file,fieldnames=FIELDS)
        self.writer.writeheader()

    def write(self,core,out,ros_s,steady_s,tcp_xy,tcp_stamp):
        p=core.plan
        row=dict(ros_s=ros_s,steady_s=steady_s,active=int(out.active),reason=out.reason,
                 plan_id=p.plan_id if p else '',revision=p.revision if p else '',
                 track_id=p.track_id if p else '',epoch=p.epoch if p else '',
                 tcp_stamp_s=tcp_stamp,tcp_x_m=tcp_xy[0],tcp_y_m=tcp_xy[1],
                 target_x_m=out.target_xy_m[0] if out.active else '',
                 target_y_m=out.target_xy_m[1] if out.active else '',
                 error_xy_m=math.hypot(*out.error_xy_m) if out.active else '',
                 cmd_vx_mps=out.velocity_xy_mps[0],cmd_vy_mps=out.velocity_xy_mps[1],
                 position_std_m=p.position_std_m if p else '',
                 release_uncertainty_ok=int(core.release_uncertainty_ok(ros_s,steady_s)))
        self.writer.writerow(row)
        self.file.flush()

    def close(self): self.file.close()

def summarize(path):
    with open(path,newline='',encoding='utf-8') as f: rows=list(csv.DictReader(f))
    active=[r for r in rows if r['active']=='1']
    errors=np.array([float(r['error_xy_m']) for r in active])
    times=np.array([float(r['steady_s']) for r in rows])
    gaps=np.diff(times)
    # Include faults, not only successful samples; never label this a G5 pass.
    reasons={r['reason']:sum(x['reason']==r['reason'] for x in rows) for r in rows}
    return dict(samples=len(rows),active_samples=len(active),inactive_samples=len(rows)-len(active),
                xy_error_p95_m=float(np.percentile(errors,95)) if len(errors) else None,
                xy_error_max_m=float(errors.max()) if len(errors) else None,
                sample_gap_p99_s=float(np.percentile(gaps,99)) if len(gaps) else None,
                nonincreasing_time_gaps=int(sum(gaps<=0)),reasons=reasons,
                note='Shadow telemetry only. TCP stamps may repeat; not a real 100 Hz feedback or G5 acceptance test.')

def main():
    import argparse
    p=argparse.ArgumentParser(); p.add_argument('csv'); args=p.parse_args()
    print(json.dumps(summarize(args.csv),indent=2,ensure_ascii=False,allow_nan=False))

if __name__=='__main__': main()
