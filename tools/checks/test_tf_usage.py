"""TF 사용 범위 정적 검사.

TF(tf2)는 고정 좌표 관계와 관측된 병뚜껑 시각화에만 쓴다. 미래 위치는 planner가 메시지로 계산한다.
문자열·주석이 아닌 실제 코드 토큰만 센다.
"""
from __future__ import annotations

import tokenize
from pathlib import Path


REPO_ROOT = Path(__file__).resolve().parents[2]
EXCLUDED_DIRS = {'build', 'install', 'test', '__pycache__'}

LOOKUP_TOKENS = {'lookup_transform', 'can_transform', 'TransformListener'}
ALLOWED_LOOKUP = {'calibration/calibration/ros_nodes.py'}
STATIC_TOKENS = {'StaticTransformBroadcaster'}
ALLOWED_STATIC = {'calibration/calibration/ros_nodes.py', 'calibration/calibration/static_frames.py'}
DYNAMIC_TOKENS = {'TransformBroadcaster', 'TFMessage'}
ALLOWED_DYNAMIC = {'tracking/tracking/tracker_node.py'}


def _sources(root: Path):
    for path in sorted((root / 'src').rglob('*.py')):
        rel = path.relative_to(root / 'src')
        if not EXCLUDED_DIRS.intersection(rel.parts):
            yield rel.as_posix(), path


def _names(path: Path) -> set[str]:
    with open(path, 'rb') as fh:
        return {tok.string for tok in tokenize.tokenize(fh.readline) if tok.type == tokenize.NAME}


def find_tf_violations(root: Path = REPO_ROOT) -> list[str]:
    violations = []
    for rel, path in _sources(root):
        names = _names(path)
        for tokens, allowed, what in ((LOOKUP_TOKENS, ALLOWED_LOOKUP, 'TF 조회'),
                                      (STATIC_TOKENS, ALLOWED_STATIC, '/tf_static 발행'),
                                      (DYNAMIC_TOKENS, ALLOWED_DYNAMIC, '/tf 발행')):
            hit = sorted(names & tokens)
            if hit and rel not in allowed:
                violations.append(f'{rel}: {what} {hit}')
        if 'TimePointZero' in names:
            violations.append(f'{rel}: TimePointZero')
    return violations


def test_tf_usage_is_confined():
    assert find_tf_violations() == []


def test_tf_usage_detects_violation(tmp_path):
    pkg = tmp_path / 'src' / 'intercept_planner' / 'intercept_planner'
    pkg.mkdir(parents=True)
    (pkg / 'bad.py').write_text('t = buf.lookup_transform("robot_base", "cap_1", future)\n'
                                '# lookup_transform in a comment is ignored\n')
    (pkg / 'ok.py').write_text('s = "TransformListener in a string is ignored"\n')
    assert find_tf_violations(tmp_path) == ["intercept_planner/intercept_planner/bad.py: TF 조회 ['lookup_transform']"]
