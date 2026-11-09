#!/usr/bin/env python3
"""pytest JUnit 결과와 REQ 추적 행렬로 하루 한 줄의 개발 지표를 기록한다."""
from __future__ import annotations

import argparse
import csv
from datetime import date, datetime
import importlib.util
import math
import os
from pathlib import Path
import subprocess
import tempfile
import xml.etree.ElementTree as ET
from zoneinfo import ZoneInfo

ROOT = Path(__file__).resolve().parents[2]
FIELDS = ("date,git_sha,unit_pass,unit_total,contract_pass,contract_total,req_link_pct,"
          "integration_stage,ros_i0_pass,ros_i0_total,unit_replay_pass,unit_replay_total,"
          "gate_status,open_issues,robot_hours").split(",")


def count_junit(paths):
    """수준이 있는 testcase만 세고 skip/error/failure는 통과에서 제외한다."""
    counts = {}
    for path in paths:
        root = ET.parse(path).getroot()
        if root.tag not in ("testsuites", "testsuite"):
            raise ValueError(f"JUnit testsuite(s)가 아님: {path}")
        for case in root.iter("testcase"):
            prop = case.find("properties/property[@name='level']")
            level = prop.get("value") if prop is not None else None
            if level not in ("U", "C", "I0", "I1"):
                continue
            passed, total = counts.get(level, (0, 0))
            ok = not any(case.find(tag) is not None for tag in ("failure", "error", "skipped"))
            counts[level] = (passed + int(ok), total + 1)
    return counts


def make_row(counts, repo, day, *, gate="", open_issues="", robot_hours="", integration_stage=""):
    date.fromisoformat(day)
    repo = Path(repo)
    spec = importlib.util.spec_from_file_location("devlog_trace", ROOT / "tools/spec/trace.py")
    trace = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(trace)
    matrix = trace.build(str(repo))
    reqs = matrix["reqs"]
    linked = sum(bool(matrix["tc_by_req"].get(req)) for req in reqs)
    row = dict.fromkeys(FIELDS, "")
    row.update(date=day, git_sha=subprocess.check_output(
        ["git", "rev-parse", "HEAD"], cwd=repo, text=True).strip(),
        req_link_pct=f"{100 * linked / len(reqs):.2f}" if reqs else "",
        gate_status=gate, open_issues=open_issues, robot_hours=robot_hours,
        integration_stage=integration_stage)
    for prefix, levels in (("unit", ("U",)), ("contract", ("C",)), ("ros_i0", ("I0",)),
                           ("unit_replay", ("U", "I1"))):
        measured = [counts[level] for level in levels if level in counts]
        if measured:
            row[prefix + "_pass"] = sum(p for p, _ in measured)
            row[prefix + "_total"] = sum(t for _, t in measured)
    return row


def write_row(path, row):
    """같은 날짜 행을 교체한다. 검증 후 임시 파일을 원자적으로 교체한다."""
    path = Path(path)
    rows = []
    if path.exists():
        with path.open(newline="", encoding="utf-8") as handle:
            reader = csv.DictReader(handle)
            if reader.fieldnames != FIELDS:
                raise ValueError("devlog CSV 헤더 불일치")
            for old in reader:
                if None in old or any(value is None for value in old.values()):
                    raise ValueError("devlog CSV 열 수 불일치")
                if old["date"] != row["date"]:
                    rows.append(old)
    rows.append(row)
    path.parent.mkdir(parents=True, exist_ok=True)
    temp = None
    try:
        with tempfile.NamedTemporaryFile(mode="w", newline="", encoding="utf-8",
                                         dir=path.parent, delete=False) as handle:
            temp = Path(handle.name)
            writer = csv.DictWriter(handle, FIELDS)
            writer.writeheader()
            writer.writerows(rows)
        os.replace(temp, path)
    finally:
        if temp is not None:
            temp.unlink(missing_ok=True)


def nonnegative_int(value):
    number = int(value)
    if number < 0:
        raise ValueError("0 이상이어야 함")
    return number


def hours(value):
    number = float(value)
    if not math.isfinite(number) or number < 0:
        raise ValueError("유한한 0 이상 시간이어야 함")
    return number


def day(value):
    if date.fromisoformat(value).isoformat() != value:
        raise ValueError("날짜는 YYYY-MM-DD")
    return value


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--junit", nargs="+", default=[])
    parser.add_argument("--date", type=day, default=datetime.now(ZoneInfo("Asia/Seoul")).date().isoformat())
    parser.add_argument("--out", type=Path, default=ROOT / "records/devlog.csv")
    parser.add_argument("--gate", default="")
    parser.add_argument("--open-issues", type=nonnegative_int, default=None)
    parser.add_argument("--robot-hours", type=hours, default=None)
    parser.add_argument("--integration-stage", choices=[f"I{i}" for i in range(6)], default="")
    args = parser.parse_args(argv)
    try:
        row = make_row(count_junit(args.junit), ROOT, args.date, gate=args.gate,
                       open_issues="" if args.open_issues is None else args.open_issues,
                       robot_hours="" if args.robot_hours is None else args.robot_hours,
                       integration_stage=args.integration_stage)
        write_row(args.out, row)
    except (ValueError, OSError, ET.ParseError, subprocess.CalledProcessError) as exc:
        parser.error(str(exc))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
