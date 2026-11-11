#!/bin/bash
set -e
cd "$HOME"
python3 - <<'PY'
import subprocess, pathlib
lines=subprocess.check_output(['ps','-eo','pid,args'],text=True).splitlines()
active=[]
for line in lines:
    parts=line.strip().split(None,1)
    if len(parts)!=2 or not parts[0].isdigit():
        continue
    try:
        cwd=(pathlib.Path('/proc')/parts[0]/'cwd').resolve()
    except (OSError, RuntimeError):
        continue
    if '/home/user/ctb-codex/' in line or cwd == pathlib.Path('/home/user/ctb-codex'):
        active.append(line)
assert not active, active
print('NO_CTB_CODEX_PROCESSES')
p=pathlib.Path.home()/'ctb-codex'
assert str(p)=='/home/user/ctb-codex',p
import shutil
shutil.rmtree(p)
assert not p.exists()
print('WSL_CTB_CODEX_REMOVED')
PY
