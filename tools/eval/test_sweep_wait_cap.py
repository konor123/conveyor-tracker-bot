"""sweep_wait_cap.py 단위 시험: 합성 검출 CSV 하나와 후보 하나로 지표와 표를 확인한다."""
import sys
from pathlib import Path

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parent))
import sweep_wait_cap as sw  # noqa: E402
from test_replay_belt_tracking import synth_csv  # noqa: E402


@pytest.mark.req('REQ-TRK-006')
@pytest.mark.level('U')
def test_candidate_metrics_and_table(tmp_path):
    path = synth_csv(tmp_path / 'a.caps.csv')
    row = sw.evaluate(0.01, 2.0, 16.0, [path], seeds=2)
    r, s = row['real'][0], row['synth']
    assert r['passes'] == 3 and r['alarms'] == 0                 # 등속 합성: 경보 없음
    assert r['recover_0.5']['ok_ratio'] > 0.9 and r['recover_0.5']['passes_any_ok'] == 3
    assert 2.0 < r['recover_0.5']['std3_ok_p50_mm'] < 4.0        # speed_std 약 1 mm/s × 3 s
    assert s['step20']['detected_1s'] == 2 and s['step10']['detected_1s'] == 0
    assert s['false_alarm_seeds'] == 0 and s['pos_p95_max_mm'] < 0.4
    table = sw.markdown([row])
    assert table.count('\n') == 3 and '| 0.01 | 2 | 16 |' in table


@pytest.mark.req('REQ-TRK-006')
@pytest.mark.level('U')
def test_cli_writes_json_and_markdown(tmp_path):
    path = synth_csv(tmp_path / 'a.caps.csv')
    assert sw.main(['--csv', str(path), '--sigma', '0.01', '--kh', '2:16', '--seeds', '1',
                    '--json', str(tmp_path / 'o.json'), '--md', str(tmp_path / 'o.md')]) == 0
    assert (tmp_path / 'o.md').read_text().startswith('| sigma_accel |')
    assert sw.main(['--merge', str(tmp_path / 'o.json'), '--json', str(tmp_path / 'm.json'),
                    '--md', str(tmp_path / 'm.md')]) == 0
    assert (tmp_path / 'm.md').read_text() == (tmp_path / 'o.md').read_text()
