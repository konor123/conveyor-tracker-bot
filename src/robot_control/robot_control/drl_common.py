"""DRL 프로그램들이 함께 쓰는 본문 조각. 그리퍼 Modbus-RTU 함수는 `drl/modbus_rtu.drl` 한 곳에만 둔다.

각 DRL 틀은 `__MODBUS_RTU__` 한 줄을 두고, 만들 때 `expand()`가 그 자리에 조각을 넣는다.
조각 본문은 2026-11-09 실기에서 검증한 일회성 그리퍼 DRL의 함수와 글자까지 같다.
"""
from __future__ import annotations

from pathlib import Path

MODBUS_RTU_PLACEHOLDER = '__MODBUS_RTU__\n'
_MODBUS_RTU = Path(__file__).with_name('drl') / 'modbus_rtu.drl'


def modbus_rtu_drl() -> str:
    """`crc16`·`read_words`·`write_checked` 정의. 호출하는 쪽이 `READ_TIMEOUT`을 먼저 정의해야 한다."""
    return _MODBUS_RTU.read_text(encoding='utf-8')


def expand(template: str) -> str:
    """틀의 `__MODBUS_RTU__` 줄을 조각으로 바꾼다. 자리가 정확히 한 번 있어야 한다."""
    if template.count(MODBUS_RTU_PLACEHOLDER) != 1:
        raise ValueError('DRL 틀에 __MODBUS_RTU__ 줄이 정확히 한 번 있어야 한다')
    return template.replace(MODBUS_RTU_PLACEHOLDER, modbus_rtu_drl())
