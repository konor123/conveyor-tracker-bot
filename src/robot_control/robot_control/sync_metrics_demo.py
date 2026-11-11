"""Generate synthetic CSV with a speed change and an explicit stop sample."""
import argparse
import numpy as np
from .belt_sync import BeltSynchronizer,PlanSample
from .sync_metrics import SyncCsv,summarize
import json

def main():
    parser=argparse.ArgumentParser(); parser.add_argument('--csv',required=True); args=parser.parse_args()
    c=BeltSynchronizer(); c.arm(1,7,2)
    cap=np.array([.35,0.]); tcp=np.array([.34,.005]); actual_v=np.zeros(2)
    writer=SyncCsv(args.csv)
    try:
        for k in range(1000):
            t=k*.01; v=np.array([.02 if t<5 else .03,0.])
            if k%3==0:
                assert c.accept(PlanSample(1,7,2,k+1,t,t,t+.2,t+.25,tuple(cap+v*.2),tuple(v)),t,t)
            out=c.step(t,t,tuple(tcp),t,'robot_base')
            writer.write(c,out,t,t,tuple(tcp),t)
            if not out.active: raise RuntimeError(out.reason)
            actual_v+=(np.array(out.velocity_xy_mps)-actual_v)*.2
            tcp+=actual_v*.01; cap+=v*.01
        writer.write(c,c.stop('DEMO_END'),10.,10.,tuple(tcp),10.)
    finally: writer.close()
    print('SYNTHETIC ONLY. Statistics include startup transient, unlike belt_sync_demo.')
    print(json.dumps(summarize(args.csv),indent=2))
if __name__=='__main__': main()
