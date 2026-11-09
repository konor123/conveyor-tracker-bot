"""일별 개발 지표의 입력·집계·저장 계약."""
import csv
import importlib.util
from pathlib import Path
import subprocess
import sys
import xml.etree.ElementTree as ET

import pytest

pytestmark = [pytest.mark.req("REQ-REC-007"), pytest.mark.level("U")]
SCRIPT = Path(__file__).with_name("devlog.py")
ROOT = SCRIPT.parents[2]


def module():
    spec = importlib.util.spec_from_file_location("devlog", SCRIPT)
    result = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(result)
    return result


def xml(path, cases):
    root = ET.Element("testsuites")
    suite = ET.SubElement(root, "testsuite")
    for level, outcome in cases:
        case = ET.SubElement(suite, "testcase", name=str(len(suite)))
        if level:
            props = ET.SubElement(case, "properties")
            ET.SubElement(props, "property", name="level", value=level)
        if outcome:
            ET.SubElement(case, outcome)
    ET.ElementTree(root).write(path)
    return path


@pytest.mark.tc("TC-REC-007-01")
def test_multiple_reports_and_unmeasured(tmp_path):
    devlog = module()
    a = xml(tmp_path / "a.xml", [("U", ""), ("U", "failure"), ("C", "error"), ("I0", "skipped")])
    b = xml(tmp_path / "b.xml", [("I1", ""), ("U", "skipped"), (None, "")])
    counts = devlog.count_junit([a, b])
    assert counts == {"U": (1, 3), "C": (0, 1), "I0": (0, 1), "I1": (1, 1)}
    row = devlog.make_row(counts, ROOT, "2026-11-09")
    assert (row["unit_replay_pass"], row["unit_replay_total"]) == (2, 4)
    empty = devlog.make_row({}, ROOT, "2026-11-09")
    assert empty["unit_pass"] == empty["ros_i0_total"] == empty["integration_stage"] == ""


@pytest.mark.tc("TC-REC-007-02")
def test_replace_all_same_day_preserves_other_days(tmp_path):
    devlog = module()
    out = tmp_path / "nested" / "devlog.csv"
    first = devlog.make_row({}, ROOT, "2026-11-08")
    second = devlog.make_row({}, ROOT, "2026-11-09")
    devlog.write_row(out, first)
    devlog.write_row(out, second)
    with out.open("a") as handle:
        csv.DictWriter(handle, devlog.FIELDS).writerow(second)
    second["gate_status"] = "M0:진행"
    devlog.write_row(out, second)
    with out.open() as handle:
        rows = list(csv.DictReader(handle))
    assert len(rows) == 2
    assert rows[0] == {k: str(v) for k, v in first.items()}
    assert rows[1]["gate_status"] == "M0:진행"


@pytest.mark.tc("TC-REC-007-03")
def test_cli_identity_trace_and_values(tmp_path):
    report = xml(tmp_path / "a.xml", [("U", "")])
    out = tmp_path / "out.csv"
    result = subprocess.run([sys.executable, str(SCRIPT), "--junit", str(report),
                             "--out", str(out), "--date", "2026-11-09",
                             "--gate", "M0:진행", "--open-issues", "2",
                             "--robot-hours", "0.5", "--integration-stage", "I0"],
                            capture_output=True, text=True)
    assert result.returncode == 0, result.stderr
    row = next(csv.DictReader(out.open()))
    assert row["git_sha"] == subprocess.check_output(["git", "rev-parse", "HEAD"], cwd=ROOT, text=True).strip()
    spec = importlib.util.spec_from_file_location("test_devlog_trace", ROOT / "tools/spec/trace.py")
    trace_module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(trace_module)
    matrix = trace_module.build(str(ROOT))
    expected = 100 * sum(bool(matrix["tc_by_req"].get(r)) for r in matrix["reqs"]) / len(matrix["reqs"])
    assert row["req_link_pct"] == f"{expected:.2f}"
    assert [row[k] for k in ("date", "gate_status", "open_issues", "robot_hours", "integration_stage")] == [
        "2026-11-09", "M0:진행", "2", "0.5", "I0"]


@pytest.mark.tc("TC-REC-007-04")
def test_real_pytest_junit_level_including_skip(tmp_path):
    (tmp_path / "conftest.py").write_text(
        f"import sys\nsys.path.insert(0, {str(ROOT / 'tools/testing')!r})\nfrom drop_pytest import *\n")
    (tmp_path / "test_sample.py").write_text(
        "import pytest\n@pytest.mark.level('U')\ndef test_ok(): pass\n"
        "@pytest.mark.level('I1')\n@pytest.mark.skip(reason='no data')\ndef test_skip(): pass\n")
    out = tmp_path / "junit.xml"
    result = subprocess.run([sys.executable, "-m", "pytest", "-q", "-p", "no:cacheprovider",
                             str(tmp_path), f"--junitxml={out}"], capture_output=True, text=True)
    assert result.returncode == 0, result.stdout + result.stderr
    levels = [c.find("properties/property[@name='level']").get("value")
              for c in ET.parse(out).iter("testcase")]
    assert levels == ["U", "I1"]


@pytest.mark.tc("TC-REC-007-05")
@pytest.mark.parametrize("args", [
    ["--date", "2026-99-99"], ["--open-issues", "-1"], ["--robot-hours", "nan"],
    ["--robot-hours", "-0.1"], ["--integration-stage", "I6"], ["--robot-hours", "inf"],
])
def test_invalid_cli_preserves_output(tmp_path, args):
    out = tmp_path / "out.csv"
    out.write_text("unchanged")
    result = subprocess.run([sys.executable, str(SCRIPT), "--out", str(out), *args], capture_output=True)
    assert result.returncode != 0
    assert out.read_text() == "unchanged"


@pytest.mark.tc("TC-REC-007-05")
def test_bad_xml_and_header_preserve_output(tmp_path):
    devlog = module()
    bad = tmp_path / "bad.xml"
    bad.write_text("<broken")
    with pytest.raises(ET.ParseError):
        devlog.count_junit([bad])
    out = tmp_path / "out.csv"
    out.write_text("wrong,header\n")
    with pytest.raises(ValueError):
        devlog.write_row(out, devlog.make_row({}, ROOT, "2026-11-09"))
    assert out.read_text() == "wrong,header\n"


@pytest.mark.tc("TC-REC-007-03")
def test_cli_unmeasured_defaults(tmp_path):
    out = tmp_path / "out.csv"
    result = subprocess.run([sys.executable, str(SCRIPT), "--out", str(out)],
                            capture_output=True, text=True)
    assert result.returncode == 0, result.stderr
    row = next(csv.DictReader(out.open()))
    for key in ("open_issues", "robot_hours", "gate_status", "unit_pass", "unit_replay_total"):
        assert row[key] == ""
    from datetime import datetime
    from zoneinfo import ZoneInfo
    assert row["date"] == datetime.now(ZoneInfo("Asia/Seoul")).date().isoformat()
