"""tools/spec/trace.py 단위 테스트."""
import importlib.util
import os
import textwrap

import pytest

_spec = importlib.util.spec_from_file_location("spec_trace", os.path.join(os.path.dirname(__file__), "trace.py"))
trace = importlib.util.module_from_spec(_spec)
_spec.loader.exec_module(trace)


def _repo(tmp_path, spec_md, test_spec_md, test_py):
    (tmp_path / "specs").mkdir()
    (tmp_path / "specs" / "SPEC-100-orchestrator.md").write_text(textwrap.dedent(spec_md), encoding="utf-8")
    (tmp_path / "TEST_SPEC.md").write_text(textwrap.dedent(test_spec_md), encoding="utf-8")
    t = tmp_path / "src" / "orchestrator" / "test"
    t.mkdir(parents=True)
    (t / "test_x.py").write_text(textwrap.dedent(test_py), encoding="utf-8")
    return str(tmp_path)


SPEC = """\
    - REQ-ORC-001: 정상 상태 열
    | REQ-ORC-002: | 판정 실패 뒤 진행 금지 |
    - REQ-ORC-003: TC 없음
"""
TS = """\
    | TC | 내용 | 수준 | 장소 |
    |---|---|---|---|
    | TC-ORC-001-01 | a | U | `[ON]` |
    | TC-ORC-002-01 | b | U | `[ON]` |
    | TC-ORC-002-02 | c | I3 | `[OFF]` |
    | TC-EXP-G2-01 | G2 | MODE_STATIC | 50 |
"""
TESTS = """\
    import pytest

    @pytest.mark.req("REQ-ORC-001")
    @pytest.mark.tc("TC-ORC-001-01")
    def test_a():
        pass

    @pytest.mark.tc("TC-ORC-009-01")
    def test_unknown():
        pass

    def test_unmarked():
        pass
"""


@pytest.mark.req("REQ-REC-006")
@pytest.mark.tc("TC-REC-006-01")
@pytest.mark.level("U")
def test_reports_missing_and_nonzero_exit(tmp_path, capsys):
    repo = _repo(tmp_path, SPEC, TS, TESTS)
    r = trace.build(repo)
    assert set(r["reqs"]) == {"REQ-ORC-001", "REQ-ORC-002", "REQ-ORC-003"}
    assert r["req_no_tc"] == ["REQ-ORC-003"]
    assert r["tc_no_test"] == ["TC-ORC-002-01"]          # I3·EXP는 절차 시험이라 제외
    assert r["by_tc"]["TC-ORC-001-01"] == ["src/orchestrator/test/test_x.py::test_a"]
    assert ("src/orchestrator/test/test_x.py::test_unknown", "TC-ORC-009-01") in r["unknown"]
    assert r["tcs"]["TC-EXP-G2-01"] == "E"
    assert trace.main(["--repo", repo]) == 1
    out = capsys.readouterr().out
    assert "REQ-ORC-003" in out and "TC-ORC-002-01" in out
    md = (tmp_path / "specs" / "TRACE.md").read_text(encoding="utf-8")
    assert "| TC-ORC-001-01 | U | REQ-ORC-001 | `src/orchestrator/test/test_x.py::test_a` |" in md


@pytest.mark.level("U")
def test_clean_repo_exits_zero(tmp_path):
    repo = _repo(tmp_path, "- REQ-ORC-001: a\n", "| TC-ORC-001-01 | a | U | x |\n",
                 "import pytest\n@pytest.mark.tc('TC-ORC-001-01')\ndef test_a():\n    pass\n")
    assert trace.main(["--repo", repo, "--check"]) == 0
    assert not os.path.exists(os.path.join(repo, "specs", "TRACE.md"))


@pytest.mark.level("U")
def test_area_filter(tmp_path):
    repo = _repo(tmp_path, SPEC, TS + "    | TC-ABT-001-01 | x | U | y |\n", TESTS)
    r = trace.build(repo, {"ABT"})
    assert list(r["tcs"]) == ["TC-ABT-001-01"] and r["reqs"] == {}


@pytest.mark.level("U")
def test_module_and_class_pytestmark_count_as_tests(tmp_path):
    body = """\
        import pytest
        from suite import *  # noqa

        pytestmark = [pytest.mark.req("REQ-ORC-001"), pytest.mark.tc("TC-ORC-001-01")]

        @pytest.mark.tc("TC-ORC-002-01")
        class TestImported(Base):
            pass
    """
    repo = _repo(tmp_path, SPEC, TS, body)
    r = trace.build(repo)
    assert r["by_tc"]["TC-ORC-001-01"] == ["src/orchestrator/test/test_x.py::<module>"]
    assert r["by_tc"]["TC-ORC-002-01"] == ["src/orchestrator/test/test_x.py::TestImported"]
