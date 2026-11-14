"""노드 진입점 종료 처리 정적 검사.

- `node_shutdown.py` 사본 7개가 바이트 단위로 같다.
- 대상 진입점은 `rclpy.init()`보다 먼저 `ShutdownGuard()`를 만들고, spin을 감싼 except가 `is_shutdown`으로
  판정한다. `KeyboardInterrupt`·`ExternalShutdownException`을 이름으로 잡는 except는 없다.
- `rclpy.try_shutdown()`마다 그 뒤에 `guard.close()`가 있다(정리 뒤 SIGINT 무시).
"""
from __future__ import annotations

import ast
import shutil
from pathlib import Path


REPO_ROOT = Path(__file__).resolve().parents[2]
COPIES = ('drop_mocks/drop_mocks', 'orchestrator/orchestrator', 'tracking/tracking',
          'intercept_planner/intercept_planner', 'robot_control/robot_control', 'vision/vision',
          'calibration/calibration')
ENTRY_POINTS = {
    'drop_mocks/drop_mocks/ros_common.py': 'spin_mock',
    'orchestrator/orchestrator/orchestrator_node.py': 'main',
    'orchestrator/orchestrator/safety_supervisor_node.py': 'main',
    'orchestrator/orchestrator/record_logger_node.py': 'main',
    'tracking/tracking/tracker_node.py': 'main',
    'intercept_planner/intercept_planner/intercept_planner_node.py': 'main',
    'robot_control/robot_control/robot_controller_node.py': 'main',
    'vision/vision/vision_node.py': 'main',
    'calibration/calibration/static_frames.py': 'main',
    'calibration/calibration/ros_nodes.py': 'camera_tf_publisher',
}
SHUTDOWN_NAMES = {'KeyboardInterrupt', 'ExternalShutdownException'}


def _call_name(node: ast.AST) -> str:
    if isinstance(node, ast.Call):
        f = node.func
        if isinstance(f, ast.Name):
            return f.id
        if isinstance(f, ast.Attribute):
            base = f.value.id if isinstance(f.value, ast.Name) else ''
            return f'{base}.{f.attr}' if base else f.attr
    return ''


def _handler_names(handler: ast.ExceptHandler) -> set[str]:
    if handler.type is None:
        return set()
    types = handler.type.elts if isinstance(handler.type, ast.Tuple) else [handler.type]
    return {t.id if isinstance(t, ast.Name) else t.attr for t in types if isinstance(t, (ast.Name, ast.Attribute))}


def find_shutdown_violations(root: Path = REPO_ROOT) -> list[str]:
    violations = []
    texts = {c: (root / 'src' / c / 'node_shutdown.py').read_bytes()
             if (root / 'src' / c / 'node_shutdown.py').exists() else None for c in COPIES}
    for c, text in texts.items():
        if text is None:
            violations.append(f'{c}/node_shutdown.py: 사본 없음')
        elif text != texts[COPIES[0]]:
            violations.append(f'{c}/node_shutdown.py: {COPIES[0]} 사본과 다름')
    for rel, func in ENTRY_POINTS.items():
        tree = ast.parse((root / 'src' / rel).read_text(encoding='utf-8'))
        fn = next((n for n in ast.walk(tree) if isinstance(n, ast.FunctionDef) and n.name == func), None)
        if fn is None:
            violations.append(f'{rel}: {func} 없음')
            continue
        calls = [(n.lineno, n.col_offset, _call_name(n)) for n in ast.walk(fn) if isinstance(n, ast.Call)]
        guard = min(((ln, col) for ln, col, name in calls if name == 'ShutdownGuard'), default=None)
        init = min(((ln, col) for ln, col, name in calls if name == 'rclpy.init'), default=None)
        if guard is None or init is None or guard >= init:
            violations.append(f'{rel}:{func}: ShutdownGuard()를 rclpy.init() 전에 만들지 않음')
        shut = [ln for ln, _, name in calls if name == 'rclpy.try_shutdown']
        close = [ln for ln, _, name in calls if name == 'guard.close']
        if not shut or any(not any(c > ln for c in close) for ln in shut):
            violations.append(f'{rel}:{func}: rclpy.try_shutdown() 뒤에 guard.close()가 없음')
        if not any(name.endswith('.is_shutdown') for _, _, name in calls):
            violations.append(f'{rel}:{func}: except가 guard.is_shutdown으로 판정하지 않음')
        for h in (n for n in ast.walk(fn) if isinstance(n, ast.ExceptHandler)):
            named = _handler_names(h) & SHUTDOWN_NAMES
            if named:
                violations.append(f'{rel}:{h.lineno}: {sorted(named)}를 이름으로 잡음')
    return violations


def test_entry_points_use_shutdown_guard():
    assert find_shutdown_violations() == []


def test_checker_reports_guard_after_init_and_diverged_copy(tmp_path):
    for rel in list(ENTRY_POINTS) + [f'{c}/node_shutdown.py' for c in COPIES]:
        dst = tmp_path / 'src' / rel
        dst.parent.mkdir(parents=True, exist_ok=True)
        shutil.copyfile(REPO_ROOT / 'src' / rel, dst)
    sup = tmp_path / 'src/orchestrator/orchestrator/safety_supervisor_node.py'
    s = sup.read_text(encoding='utf-8')
    guard_line = '    guard = ShutdownGuard()            # rclpy.init 전\n'
    init_line = '    rclpy.init(args=args)\n'
    assert s.count(guard_line + init_line) == 1
    sup.write_text(s.replace(guard_line + init_line, init_line + guard_line), encoding='utf-8')
    with open(tmp_path / 'src/tracking/tracking/node_shutdown.py', 'a', encoding='utf-8') as fh:
        fh.write('\n')
    planner = tmp_path / 'src/intercept_planner/intercept_planner/intercept_planner_node.py'
    t = planner.read_text(encoding='utf-8')
    assert t.count('        guard.close()\n') == 1
    planner.write_text(t.replace('        guard.close()\n', ''), encoding='utf-8')
    got = find_shutdown_violations(tmp_path)
    assert got == ['tracking/tracking/node_shutdown.py: drop_mocks/drop_mocks 사본과 다름',
                   'orchestrator/orchestrator/safety_supervisor_node.py:main: '
                   'ShutdownGuard()를 rclpy.init() 전에 만들지 않음',
                   'intercept_planner/intercept_planner/intercept_planner_node.py:main: '
                   'rclpy.try_shutdown() 뒤에 guard.close()가 없음']
