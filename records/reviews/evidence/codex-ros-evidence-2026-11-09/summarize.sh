#!/bin/bash
set -e
cd ~/ctb-codex
python3 - <<'PY' > /mnt/c/tmp/codex-w7/nominal-summary.json
from pathlib import Path
import json,hashlib
summary=[]
for n in range(1,9):
 root=Path('nominal-results')/str(n)
 paths=list(root.rglob('trials.jsonl'))
 assert len(paths)==1, (n,paths)
 p=paths[0]; rows=[json.loads(x) for x in p.read_text().splitlines()]
 assert len(rows)==20, (n,len(rows))
 assert [r['scenario']['seed'] for r in rows]==list(range(20))
 assert all(r['result']=='SUCCESS' and r['abort_level']==0 for r in rows)
 steps=sorted(p.parent.glob('*.steps.jsonl'))
 assert len(steps)==20, (n,len(steps))
 counts=[len(q.read_text().splitlines()) for q in steps]
 assert counts==[12]*20, (n,counts)
 backward=sum(json.loads(line)['t_end_s'] < json.loads(line)['t_start_s'] for q in steps for line in q.read_text().splitlines())
 summary.append(dict(run=n,trials=20,success=20,step_records=sum(counts),backward_ros_steps=backward,sha256=hashlib.sha256(p.read_bytes()).hexdigest()))
print(json.dumps(dict(runs=summary,cycles=160,success=160,step_records=1920),indent=2))
PY
tar czf /mnt/c/tmp/codex-w7/nominal-results.tar.gz nominal-results
sha256sum /mnt/c/tmp/codex-w7/nominal-results.tar.gz
