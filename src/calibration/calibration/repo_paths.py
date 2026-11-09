"""설정 파일 경로 규칙(SPEC-000 4.4절, REQ-SYS-005): 상대 경로는 저장소 루트 기준이다.

저장소 루트를 찾는 순서:
1. 환경 변수 `DROP_REPO_ROOT`
2. 이 모듈 위치에서 위로 올라가며 `src`와 `records` 폴더를 함께 가진 첫 폴더
   (소스 실행, colcon `--symlink-install`, 일반 install 모두 워크스페이스 = 저장소 루트 아래에 있다)
3. 현재 작업 폴더
"""
from __future__ import annotations

import os
from pathlib import Path

ENV_REPO_ROOT = 'DROP_REPO_ROOT'


def repo_root(start: str | os.PathLike | None = None) -> Path:
    env = os.environ.get(ENV_REPO_ROOT)
    if env:
        return Path(env).expanduser().resolve()
    here = Path(start if start is not None else __file__).resolve()
    for d in [here, *here.parents]:
        if (d / 'src').is_dir() and (d / 'records').is_dir():
            return d
    return Path.cwd()


def resolve_path(value: str | os.PathLike | None, root: str | os.PathLike | None = None) -> str:
    """빈 값은 빈 문자열 그대로, `~`는 홈으로, 절대 경로는 그대로, 상대 경로는 저장소 루트 기준 절대 경로로."""
    if not value:
        return ''
    p = Path(os.path.expanduser(str(value)))
    if p.is_absolute():
        return str(p)
    return os.path.normpath(str(Path(root) if root is not None else repo_root()) + os.sep + str(p))
