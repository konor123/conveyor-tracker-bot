#!/usr/bin/env python3
"""One-shot gripper control via the existing controller flange RS-485 path."""
import argparse
from datetime import datetime
from pathlib import Path
import subprocess
import sys

HERE = Path(__file__).resolve().parent


def render(pulse, current):
    if type(pulse) is not int or not 0 <= pulse <= 750:
        raise ValueError('pulse must be an integer in 0..750')
    if type(current) is not int or not 1 <= current <= 100:
        raise ValueError('current must be an integer in 1..100 mA (approved limit)')
    return (HERE / 'drl/gripper_once.drl').read_text().replace('__PULSE__', str(pulse)).replace('__CURRENT__', str(current))


def main(argv=None):
    parser = argparse.ArgumentParser(description='그리퍼 일회성 DRL 제어. 현장 비상정지 담당자·간섭 확인 필수. 하드웨어 pulse 사용.')
    parser.add_argument('action', choices=['open', 'close', 'set'])
    parser.add_argument('--pulse', type=int, help='목표 pulse (0..750), 생략 시 open=0, close=750')
    parser.add_argument('--current', type=int, default=100, help='전류 1..100 mA (기본 100)')
    parser.add_argument('--dry-run', action='store_true', help='장비에 연결하지 않고 DRL 출력')
    args = parser.parse_args(argv)
    pulse = args.pulse
    if pulse is None:
        if args.action == 'set':
            parser.error('set requires --pulse')
        pulse = 0 if args.action == 'open' else 750
    try:
        code = render(pulse, args.current)
    except ValueError as exc:
        parser.error(str(exc))
    if args.dry_run:
        print(code)
        return 0
    destination = HERE.parents[1] / 'records/bench/gripper-cli'
    destination.mkdir(parents=True, exist_ok=True)
    stamp = datetime.now().astimezone().strftime('%Y%m%dT%H%M%S%f%z')
    drl = destination / (stamp + '.drl')
    drl.write_text(code)
    print(f'그리퍼 목표 {pulse} pulse, {args.current} mA. DRL: {drl}', flush=True)
    with (destination / (stamp + '.jsonl')).open('w') as log:
        process = subprocess.Popen([sys.executable, str(HERE / 'drl/gripper_timing_run.py'), str(drl), '--timeout', '60'], stdout=subprocess.PIPE, text=True)
        try:
            for line in process.stdout:
                print(line, end='', flush=True)
                log.write(line)
                log.flush()
            return process.wait()
        except KeyboardInterrupt:
            process.wait()
            print('호스트 감시 중단. 장비 정지를 의미하지 않습니다. 현장 상태를 확인하세요.', file=sys.stderr)
            return 130


if __name__ == '__main__':
    sys.exit(main())
