"""drop_ui.py 순수 함수 단위 시험(창을 띄우지 않는다)."""
import pytest

from tools.robot.drop_ui import ABORT_TEXT, Tally, build_command, describe


def test_commands_for_the_three_buttons():
    one = build_command('one', 'site.yaml', record='rec/ui-1', stop_file='rec/STOP')
    assert one[2:] == ['--config', 'site.yaml', '--execute', '--count', '1', '--speed-scale', '1',
                       '--stop-file', 'rec/STOP', '--record', 'rec/ui-1']
    every = build_command('all', 'site.yaml', speed=1.5, holding=True)
    assert every[2:] == ['--config', 'site.yaml', '--execute', '--count', '0', '--speed-scale', '1.5', '--holding']
    assert build_command('park', 'site.yaml', speed=2.0, record='x', holding=True)[2:] == [
        '--config', 'site.yaml', '--execute', '--park-only']                 # 복귀는 집기·녹화 옵션을 받지 않는다
    with pytest.raises(ValueError):
        build_command('home', 'site.yaml')


def test_events_are_described_in_korean_and_noise_is_dropped():
    assert describe({'event': 'stone', 'pos_m': [0.6505, -0.1118, -0.018]}) == '공깃돌 찾음 (x 650, y -112 mm)'
    assert describe({'event': 'done', 'step': 'wait'}) == '벨트 옆에서 뚜껑을 기다리는 중'
    assert describe({'event': 'plan', 'cap_v_mps': [0.0407, 0.0], 'release_in_s': 2.9}) == '뚜껑 보임: 41 mm/s, 2.9초 뒤 도착'
    assert describe({'event': 'verdict', 'index': 2, 'result': 'SUCCESS'}) == '2번째 판정: 성공'
    assert describe({'event': 'abort', 'reason': 'NO_MAT_STONE'}) == '중단: ' + ABORT_TEXT['NO_MAT_STONE']
    assert describe({'event': 'abort', 'reason': 'SOMETHING_NEW'}) == '중단: SOMETHING_NEW'
    assert describe({'event': 'summary', 'moved': 0, 'parked': True}) == '원위치로 돌아왔습니다'
    assert describe({'event': 'summary', 'moved': 3, 'unreachable': 2}) == '끝: 3개 놓음 (닿지 않는 돌 2개 남음)'
    assert describe({'event': 'unreachable', 'pos_m': [[0.706, -0.093, -0.019]]}) == '팔이 닿지 않는 공깃돌 1개는 건너뜁니다'
    assert describe({'event': 'abort', 'reason': 'STONE_OUT_OF_REACH'}).startswith('중단: 남은 공깃돌이 팔이 닿지 않는')
    assert describe({'event': 'check', 'step': 'wait', 'ok': True}) is None
    assert describe({'event': 'done', 'step': 'unknown_step'}) is None


def test_tally_counts_releases_verdicts_and_keeps_the_last_reason():
    tally = Tally()
    for event in ({'event': 'released', 'index': 1}, {'event': 'verdict', 'index': 1, 'result': 'SUCCESS'},
                  {'event': 'released', 'index': 2}, {'event': 'verdict', 'index': 2, 'result': 'MISS'},
                  {'event': 'released', 'index': 3}, {'event': 'verdict', 'index': 3, 'result': 'UNKNOWN'},
                  {'event': 'abort', 'reason': 'NOT_RELEASED'}, {'event': 'summary', 'moved': 3, 'reason': 'NOT_RELEASED'}):
        tally.feed(event)
    assert tally.text() == '놓음 3 · 성공 1 · 실패 1 · 미확인 1'
    assert tally.last_reason == 'NOT_RELEASED'


def test_stop_and_place_back_are_described_and_holding_comes_from_the_summary():
    assert describe({'event': 'stop_seen', 'where': 'wait_cap'}) == '중지 요청 확인'
    assert describe({'event': 'done', 'step': 'place_back_lower'}) == '쥔 돌을 매트에 내려놓는 중'
    assert describe({'event': 'summary', 'moved': 2, 'holding': True}) == '끝: 2개 놓음 · 돌을 쥔 채 원위치'
    tally = Tally()
    tally.feed({'event': 'abort', 'reason': 'STOP_REQUESTED'})
    tally.feed({'event': 'summary', 'moved': 2, 'reason': 'STOP_REQUESTED', 'holding': False, 'placed_back': True})
    assert tally.holding is False and tally.last_reason == 'STOP_REQUESTED'
