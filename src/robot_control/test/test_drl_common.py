"""DRL 공통 조각(`drl_common`): Modbus-RTU 함수는 한 곳에만 있고, 틀마다 자리 표시가 정확히 한 번 있다."""
from pathlib import Path

import pytest

from robot_control import drl_common
from robot_control.resident_transport import render_resident_drl

SRC = Path(__file__).resolve().parents[2]
TEMPLATES = [SRC / 'robot_control' / 'robot_control' / 'drl' / 'gripper_resident.drl',
             SRC / 'static_pick_place' / 'static_pick_place' / 'drl' / 'gripper_move.drl',
             SRC / 'static_pick_place' / 'static_pick_place' / 'drl' / 'cycle.drl']
FUNCTIONS = ('def crc16(d):', 'def read_words(addr, count):', 'def write_checked(pdu):')


def test_snippet_defines_the_modbus_functions_once():
    snippet = drl_common.modbus_rtu_drl()
    assert all(snippet.count(f) == 1 for f in FUNCTIONS)
    assert '__' not in snippet and snippet.endswith('\n')


@pytest.mark.parametrize('path', TEMPLATES, ids=lambda p: p.name)
def test_templates_hold_one_placeholder_and_no_copy_of_the_functions(path):
    template = path.read_text(encoding='utf-8')
    assert template.count(drl_common.MODBUS_RTU_PLACEHOLDER) == 1
    assert not any(f in template for f in FUNCTIONS)          # 본문 복사본을 다시 넣지 않는다
    expanded = drl_common.expand(template)
    assert all(expanded.count(f) == 1 for f in FUNCTIONS)
    # 조각은 READ_TIMEOUT을 쓰므로 그 정의 뒤에 들어가야 한다
    assert expanded.index('READ_TIMEOUT = ') < expanded.index(FUNCTIONS[0])


def test_expand_rejects_a_template_without_exactly_one_placeholder():
    for bad in ('x = 1\n', drl_common.MODBUS_RTU_PLACEHOLDER * 2):
        with pytest.raises(ValueError):
            drl_common.expand(bad)
    assert '__MODBUS_RTU__' not in render_resident_drl()
