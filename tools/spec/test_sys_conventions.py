"""SYS 규약 단위 테스트 (TEST_SPEC.md 3.1절: TC-SYS-001-01, TC-SYS-002-01, TC-SYS-004-01)."""
from __future__ import annotations

import glob
import io
import os
import subprocess
import tokenize
import xml.etree.ElementTree as ET
from pathlib import Path

import pytest
import yaml

REPO_ROOT = Path(__file__).resolve().parents[2]
ROS_SETUP = Path("/opt/ros/jazzy/setup.bash")
UNIT_SUFFIXES = (
    "_m",
    "_s",
    "_mps",
    "_mps2",
    "_rad",
    "_hz",
    "_ma",
    "_pulse",
    "_count",
    "_ratio",
)
ALLOWED_UNIT_ADAPTERS = {"motion_adapter.py", "gripper_adapter.py"}
EXCLUDED_DIRS = {"build", "install", "test", "__pycache__"}


def _clean_underlay_env() -> dict[str, str]:
    """워크스페이스 경로(AMENT_PREFIX_PATH·COLCON_PREFIX_PATH·PYTHONPATH 등)를 제거한 깨끗한 환경."""
    env: dict[str, str] = {
        "PATH": "/usr/local/sbin:/usr/local/bin:/usr/sbin:/usr/bin:/sbin:/bin",
        "HOME": os.environ.get("HOME", "/tmp"),
        "LANG": os.environ.get("LANG", "C.UTF-8"),
    }
    if "LC_ALL" in os.environ:
        env["LC_ALL"] = os.environ["LC_ALL"]
    return env


def check_underlay_package_collisions(
    root: Path = REPO_ROOT,
    setup_bash: Path = ROS_SETUP,
) -> tuple[list[str], list[str]]:
    """src/*/package.xml의 <name> 각각에 대해 underlay에서 ros2 pkg prefix <name> 실패 여부를 확인한다."""
    pkg_xmls = sorted((root / "src").glob("*/package.xml"))
    pkg_names: list[str] = []
    for xml_path in pkg_xmls:
        name = ET.parse(xml_path).findtext("name")
        if name and name.strip():
            pkg_names.append(name.strip())

    env = _clean_underlay_env()
    violations: list[str] = []
    for name in pkg_names:
        proc = subprocess.run(
            ["bash", "-c", f"source {setup_bash} && ros2 pkg prefix {name}"],
            env=env,
            capture_output=True,
            text=True,
            check=False,
        )
        combined = f"{proc.stdout}\n{proc.stderr}".strip()
        if proc.returncode == 0 or "not found" not in combined.lower():
            violations.append(f"{name}: rc={proc.returncode}, output={combined!r}")
    return pkg_names, violations


def _is_numeric_value(val: object) -> bool:
    """bool을 제외한 수치 스칼라 또는 수치 목록인지 판정한다."""
    if isinstance(val, bool):
        return False
    if isinstance(val, (int, float)):
        return True
    if isinstance(val, list) and val:
        return all(_is_numeric_value(item) for item in val)
    return False


def find_yaml_unit_violations(root: Path = REPO_ROOT) -> list[str]:
    """src/*/config/*.yaml, src/*/scenarios/*.yaml에서 단위 접미사 없는 수치 키를 찾는다."""
    patterns = [
        str(root / "src" / "*" / "config" / "*.yaml"),
        str(root / "src" / "*" / "scenarios" / "*.yaml"),
    ]
    yaml_files = sorted({Path(p) for pat in patterns for p in glob.glob(pat)})
    violations: list[str] = []

    for yaml_path in yaml_files:
        rel_file = yaml_path.relative_to(root).as_posix()
        with open(yaml_path, encoding="utf-8") as fh:
            doc = yaml.safe_load(fh)
        if not isinstance(doc, dict):
            continue

        raw_unitless = doc.get("unitless") or []
        unitless = {str(x) for x in raw_unitless} if isinstance(raw_unitless, list) else set()

        def _walk(node: object, prefix: str = "") -> None:
            if isinstance(node, dict):
                for k, v in node.items():
                    key = str(k)
                    if prefix == "" and key in ("unitless", "measured"):
                        continue
                    key_path = f"{prefix}.{key}" if prefix else key
                    if _is_numeric_value(v):
                        if (
                            key not in unitless
                            and key_path not in unitless
                            and not key.endswith(UNIT_SUFFIXES)
                        ):
                            violations.append(f"{rel_file}:{key_path}")
                    elif isinstance(v, (dict, list)):
                        _walk(v, key_path)
            elif isinstance(node, list):
                for item in node:
                    if isinstance(item, (dict, list)):
                        _walk(item, prefix)

        _walk(doc)

    return violations


def _is_thousand_token(tok: tokenize.TokenInfo) -> bool:
    if tok.type != tokenize.NUMBER:
        return False
    try:
        return float(tok.string.replace("_", "")) == 1000.0
    except ValueError:
        return False


def find_unit_conversion_violations(root: Path = REPO_ROOT) -> list[str]:
    """src/**/*.py(build/install/test 제외)에서 adapter 외 단위 변환 패턴을 정적 검사한다."""
    src_root = root / "src"
    violations: list[str] = []

    for py_path in sorted(src_root.rglob("*.py")):
        rel_parts = py_path.relative_to(src_root).parts
        if any(part in EXCLUDED_DIRS for part in rel_parts[:-1]):
            continue
        if py_path.name in ALLOWED_UNIT_ADAPTERS:
            continue

        rel_file = py_path.relative_to(root).as_posix()
        source = py_path.read_text(encoding="utf-8")
        code_tokens = [
            tok
            for tok in tokenize.generate_tokens(io.StringIO(source).readline)
            if tok.type in (tokenize.NAME, tokenize.OP, tokenize.NUMBER)
        ]

        for i, tok in enumerate(code_tokens):
            if tok.type == tokenize.OP and tok.string in ("*", "/", "*=", "/="):
                if i + 1 < len(code_tokens) and _is_thousand_token(code_tokens[i + 1]):
                    nxt = code_tokens[i + 1]
                    violations.append(f"{rel_file}:{tok.start[0]}:{tok.string} {nxt.string}")
            elif (
                tok.type == tokenize.NUMBER
                and _is_thousand_token(tok)
                and i + 1 < len(code_tokens)
                and code_tokens[i + 1].type == tokenize.OP
                and code_tokens[i + 1].string == "*"
            ):
                op_tok = code_tokens[i + 1]
                violations.append(f"{rel_file}:{tok.start[0]}:{tok.string} {op_tok.string}")
            elif (
                tok.type == tokenize.NAME
                and tok.string == "math"
                and i + 2 < len(code_tokens)
                and code_tokens[i + 1].type == tokenize.OP
                and code_tokens[i + 1].string == "."
                and code_tokens[i + 2].type == tokenize.NAME
                and code_tokens[i + 2].string in ("degrees", "radians")
            ):
                fn_tok = code_tokens[i + 2]
                violations.append(f"{rel_file}:{tok.start[0]}:math.{fn_tok.string}")

    return violations


@pytest.mark.req("REQ-SYS-001")
@pytest.mark.tc("TC-SYS-001-01")
@pytest.mark.level("U")
def test_package_names_do_not_collide_with_underlay():
    if not ROS_SETUP.exists():
        pytest.skip("/opt/ros/jazzy가 없어 건너뜀")
    pkg_names, violations = check_underlay_package_collisions(REPO_ROOT, ROS_SETUP)
    assert len(pkg_names) == 11, f"패키지 수가 11개가 아님: {pkg_names}"
    assert not violations, "underlay와 겹치는 패키지:\n" + "\n".join(violations)


@pytest.mark.req("REQ-SYS-002")
@pytest.mark.tc("TC-SYS-002-01")
@pytest.mark.level("U")
def test_yaml_numeric_keys_have_unit_suffix():
    violations = find_yaml_unit_violations(REPO_ROOT)
    assert not violations, "단위 접미사 없는 수치 키(파일:키):\n" + "\n".join(violations)


@pytest.mark.req("REQ-SYS-004")
@pytest.mark.tc("TC-SYS-004-01")
@pytest.mark.level("U")
def test_unit_conversions_only_in_adapters():
    violations = find_unit_conversion_violations(REPO_ROOT)
    assert not violations, "허용된 adapter 외 단위 변환 패턴:\n" + "\n".join(violations)


@pytest.mark.level("U")
def test_underlay_collision_detector_catches_violation(tmp_path):
    if not ROS_SETUP.exists():
        pytest.skip("/opt/ros/jazzy가 없어 건너뜀")
    pkg_dir = tmp_path / "src" / "rclpy"
    pkg_dir.mkdir(parents=True)
    (pkg_dir / "package.xml").write_text(
        "<package format='3'><name>rclpy</name></package>\n",
        encoding="utf-8",
    )
    pkg_names, violations = check_underlay_package_collisions(tmp_path, ROS_SETUP)
    assert pkg_names == ["rclpy"]
    assert len(violations) == 1 and "rclpy" in violations[0]


@pytest.mark.level("U")
def test_yaml_unit_checker_catches_violation(tmp_path):
    cfg_dir = tmp_path / "src" / "sample_pkg" / "config"
    scen_dir = tmp_path / "src" / "sample_pkg" / "scenarios"
    cfg_dir.mkdir(parents=True)
    scen_dir.mkdir(parents=True)
    (cfg_dir / "bad.yaml").write_text(
        "unitless: [seed]\n"
        "seed: 0\n"
        "enabled: true\n"
        "timeout_s: 1.5\n"
        "bad_scalar: 42\n"
        "bad_list: [1.0, 2.0]\n"
        "measured:\n"
        "  raw_uncheck_key: 'citation'\n",
        encoding="utf-8",
    )
    violations = find_yaml_unit_violations(tmp_path)
    assert violations == [
        "src/sample_pkg/config/bad.yaml:bad_scalar",
        "src/sample_pkg/config/bad.yaml:bad_list",
    ]


@pytest.mark.level("U")
def test_unit_conversion_checker_catches_violation(tmp_path):
    pkg_dir = tmp_path / "src" / "sample_pkg" / "sample_pkg"
    test_dir = tmp_path / "src" / "sample_pkg" / "test"
    pkg_dir.mkdir(parents=True)
    test_dir.mkdir(parents=True)

    # 주석·docstring·허용 adapter·test 디렉터리는 제외되어야 함
    (pkg_dir / "motion_adapter.py").write_text(
        "import math\nx = 1.0 * 1000\ny = math.degrees(0.5)\n",
        encoding="utf-8",
    )
    (test_dir / "test_ignore.py").write_text(
        "import math\nx = 1.0 / 1000\ny = math.radians(90)\n",
        encoding="utf-8",
    )
    (pkg_dir / "bad_module.py").write_text(
        '"""Docstring with * 1000 and math.degrees(x)."""\n'
        "# Comment with / 1000 and math.radians(y)\n"
        "import math\n"
        "a = dist_m * 1000\n"
        "b = dist_mm / 1000\n"
        "c = math.degrees(rad)\n"
        "d = math.radians(deg)\n",
        encoding="utf-8",
    )
    violations = find_unit_conversion_violations(tmp_path)
    assert violations == [
        "src/sample_pkg/sample_pkg/bad_module.py:4:* 1000",
        "src/sample_pkg/sample_pkg/bad_module.py:5:/ 1000",
        "src/sample_pkg/sample_pkg/bad_module.py:6:math.degrees",
        "src/sample_pkg/sample_pkg/bad_module.py:7:math.radians",
    ]
