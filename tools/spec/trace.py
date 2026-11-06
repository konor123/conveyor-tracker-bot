#!/usr/bin/env python3
"""REQ ↔ TC ↔ 테스트 추적 행렬(PLAN.md 8.3절, TC-REC-006-01).

  python3 tools/spec/trace.py                  # specs/TRACE.md 갱신, 누락이 있으면 종료 코드 1
  python3 tools/spec/trace.py --area ORC,ABT   # 영역 제한(게이트별 확인)
  python3 tools/spec/trace.py --check          # 파일을 쓰지 않고 검사만

입력:
  REQ  specs/SPEC-*.md 안의 `REQ-<영역>-<번호>` 정의(줄 머리 또는 표 첫 칸의 `REQ-...:`)
  TC   TEST_SPEC.md 표의 첫 칸 `TC-<영역>-<번호>-<순번>`과 수준 열
  테스트 src/*/test/**/*.py, tools/**/test_*.py 안의 @pytest.mark.tc("...")·req("...")

pytest로 돌리는 수준(U, C, I0, I1, I2)의 TC만 "테스트 없음" 검사 대상이다. S·I3~I5·B·E는 절차 시험이라 제외한다.
"""
from __future__ import annotations

import argparse
import ast
import glob
import os
import re
import sys
from collections import defaultdict

PYTEST_LEVELS = {"U", "C", "I0", "I1", "I2"}
REQ_DEF = re.compile(r"(?:^\s*(?:[-*]\s+)?|\|\s*)\**(REQ-[A-Z]{2,4}-\d{3})\**\s*[:|]")
TC_ROW = re.compile(r"^\|\s*(TC-[A-Z]{2,4}-(?:\d{3}|G\d|CMP)-\d{2})\s*\|(.*)$")


def tc_req(tc: str) -> str | None:
    """TC-ABT-003-01 → REQ-ABT-003. 게이트 실험(TC-EXP-G2-01 등)은 REQ 번호가 없어 None."""
    _, area, num, _ = tc.split("-")
    return f"REQ-{area}-{num}" if num.isdigit() else None


def area_of(x: str) -> str:
    return x.split("-")[1]


def load_reqs(repo: str) -> dict[str, str]:
    reqs = {}
    for f in sorted(glob.glob(os.path.join(repo, "specs", "SPEC-*.md"))):
        for line in open(f, encoding="utf-8"):
            for m in REQ_DEF.finditer(line):
                reqs.setdefault(m.group(1), os.path.relpath(f, repo))
    return reqs


def load_tcs(repo: str, path: str = "TEST_SPEC.md") -> dict[str, str]:
    """TC ID → 수준."""
    tcs = {}
    for line in open(os.path.join(repo, path), encoding="utf-8"):
        m = TC_ROW.match(line)
        if not m:
            continue
        cells = [c.strip() for c in m.group(2).split("|")]
        level = next((c for c in cells if c in PYTEST_LEVELS | {"S", "I3", "I4", "I5", "B", "E"}), "")
        if m.group(1).startswith("TC-BEN-"):
            level = "B"
        elif m.group(1).startswith("TC-EXP-"):
            level = "E"
        tcs[m.group(1)] = level
    return tcs


def _mark_ids(dec) -> tuple[str, list[str]] | None:
    """@pytest.mark.tc("...") / req("...") 데코레이터면 (종류, ID 목록)."""
    if not isinstance(dec, ast.Call) or not isinstance(dec.func, ast.Attribute):
        return None
    f = dec.func
    if f.attr not in ("tc", "req") or not isinstance(f.value, ast.Attribute) or f.value.attr != "mark":
        return None
    ids = [a.value for a in dec.args if isinstance(a, ast.Constant) and isinstance(a.value, str)]
    return f.attr, ids


def load_tests(repo: str) -> list[tuple[str, list[str], list[str]]]:
    """(테스트 위치, TC 목록, REQ 목록). 함수 데코레이터만 읽는다(문자열 안의 마커는 무시)."""
    files = glob.glob(os.path.join(repo, "src", "*", "test", "**", "*.py"), recursive=True)
    files += glob.glob(os.path.join(repo, "tools", "**", "test_*.py"), recursive=True)
    out = []
    for f in sorted(set(files)):
        tree = ast.parse(open(f, encoding="utf-8").read(), filename=f)
        for node in ast.walk(tree):
            if not isinstance(node, (ast.FunctionDef, ast.AsyncFunctionDef)) or not node.name.startswith("test"):
                continue
            got = {"tc": [], "req": []}
            for dec in node.decorator_list:
                m = _mark_ids(dec)
                if m:
                    got[m[0]] += m[1]
            if got["tc"] or got["req"]:
                out.append((f"{os.path.relpath(f, repo)}::{node.name}", got["tc"], got["req"]))
    return out


def build(repo: str, areas: set[str] | None = None, tc_file: str = "TEST_SPEC.md"):
    reqs, tcs, tests = load_reqs(repo), load_tcs(repo, tc_file), load_tests(repo)
    keep = (lambda x: area_of(x) in areas) if areas else (lambda x: True)
    reqs = {k: v for k, v in reqs.items() if keep(k)}
    tcs = {k: v for k, v in tcs.items() if keep(k)}
    by_tc = defaultdict(list)
    unknown = []
    for name, tlist, rlist in tests:
        for t in tlist:
            if t in tcs or not keep(t):
                by_tc[t].append(name)
            else:
                unknown.append((name, t))
        for r in rlist:
            if keep(r) and r not in reqs and reqs:
                unknown.append((name, r))
    tc_by_req = defaultdict(list)
    for t in tcs:
        if tc_req(t):
            tc_by_req[tc_req(t)].append(t)
    req_no_tc = sorted(r for r in reqs if not tc_by_req.get(r))
    tc_no_test = sorted(t for t, lv in tcs.items() if lv in PYTEST_LEVELS and not by_tc.get(t))
    tc_no_req = sorted(t for t in tcs if reqs and tc_req(t) and tc_req(t) not in reqs)
    return dict(reqs=reqs, tcs=tcs, by_tc=by_tc, tc_by_req=tc_by_req, req_no_tc=req_no_tc,
                tc_no_test=tc_no_test, tc_no_req=tc_no_req, unknown=unknown)


def render(r) -> str:
    lines = ["# TRACE — REQ ↔ TC ↔ 테스트", "",
             "`tools/spec/trace.py`가 생성한다. 손으로 고치지 않는다.", ""]
    n_req, n_ok = len(r["reqs"]), sum(1 for q in r["reqs"] if r["tc_by_req"].get(q))
    pyt = [t for t, lv in r["tcs"].items() if lv in PYTEST_LEVELS]
    n_tested = sum(1 for t in pyt if r["by_tc"].get(t))
    lines += [f"- REQ {n_req}개 중 TC 연결 {n_ok}개", f"- pytest 대상 TC {len(pyt)}개 중 테스트 연결 {n_tested}개", ""]
    lines += ["| TC | 수준 | REQ | 테스트 |", "|---|---|---|---|"]
    for t in sorted(r["tcs"]):
        req = tc_req(t)
        req_s = "—" if req is None else (req if req in r["reqs"] else f"{req}(SPEC 없음)")
        tests = "<br>".join(f"`{x}`" for x in r["by_tc"].get(t, [])) or ("—" if r["tcs"][t] in PYTEST_LEVELS else "절차 시험")
        lines.append(f"| {t} | {r['tcs'][t]} | {req_s} | {tests} |")
    for title, key in (("TC가 없는 REQ", "req_no_tc"), ("테스트가 없는 TC(pytest 대상)", "tc_no_test")):
        lines += ["", f"## {title}", ""] + ([f"- {x}" for x in r[key]] or ["- 없음"])
    if r["unknown"]:
        lines += ["", "## 명세에 없는 ID를 단 테스트", ""] + [f"- `{n}`: {i}" for n, i in r["unknown"]]
    return "\n".join(lines) + "\n"


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--repo", default=os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "..")))
    ap.add_argument("--area", default="", help="쉼표 구분 영역(예: ORC,ABT)")
    ap.add_argument("--check", action="store_true", help="specs/TRACE.md를 쓰지 않음")
    ap.add_argument("--tc-file", default="TEST_SPEC.md")
    a = ap.parse_args(argv)
    areas = {x.strip() for x in a.area.split(",") if x.strip()} or None
    r = build(a.repo, areas, a.tc_file)
    if not a.check:
        os.makedirs(os.path.join(a.repo, "specs"), exist_ok=True)
        with open(os.path.join(a.repo, "specs", "TRACE.md"), "w", encoding="utf-8") as fh:
            fh.write(render(r))
    print(f"REQ {len(r['reqs'])} / TC {len(r['tcs'])} / TC 없는 REQ {len(r['req_no_tc'])} / "
          f"테스트 없는 TC {len(r['tc_no_test'])} / 명세에 없는 ID {len(r['unknown'])}")
    for title, key in (("TC 없는 REQ", "req_no_tc"), ("테스트 없는 TC", "tc_no_test")):
        if r[key]:
            print(f"{title}: " + ", ".join(r[key]))
    for n, i in r["unknown"]:
        print(f"명세에 없는 ID: {n} → {i}")
    return 1 if (r["req_no_tc"] or r["tc_no_test"] or r["unknown"]) else 0


if __name__ == "__main__":
    sys.exit(main())
