"""speedscope(py-spy) 기록에서 publish 안에 머문 가장 긴 연속 구간의 스택과 같은 순간 다른 스레드 스택을 출력한다."""
import json, sys
d = json.load(open(sys.argv[1]))
frames = d['shared']['frames']
name = lambda i: frames[i]['name'] + (' [' + frames[i].get('file', '').split('/')[-1] + ']' if frames[i].get('file') else '')
profs = d['profiles']
def runs(p):
    best, cur, start = (0, 0), 0, 0
    for k, st in enumerate(p['samples']):
        if any(frames[i]['name'] == 'Publisher.publish' for i in st):
            if cur == 0: start = k
            cur += 1
            if cur > best[0]: best = (cur, start)
        else:
            cur = 0
    return best
for p in profs:
    n, start = runs(p)
    if n >= 10:
        print(f"== {p['name']}: publish 안 연속 표본 {n}개(시작 {start}, 전체 {len(p['samples'])})")
        mid = p['samples'][start + n // 2]
        for i in mid: print('   ', name(i))
        others = [q for q in profs if q is not p and len(q['samples']) > start + n // 2]
        for q in others:
            st = q['samples'][start + n // 2]
            print(f"-- 같은 순서의 {q['name']}: " + ' <- '.join(name(i) for i in reversed(st[-6:])))
# 연속 구간 안 표본들의 가장 깊은 native 프레임 분포
import collections
for p in profs:
    n, start = runs(p)
    if n >= 10:
        c = collections.Counter(' <- '.join(name(i) for i in reversed(p['samples'][k][-4:])) for k in range(start, start + n))
        print('== 가장 깊은 4프레임 분포:')
        for k, v in c.most_common(6): print(f'   {v:3d}  {k}')
