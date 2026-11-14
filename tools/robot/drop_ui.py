#!/usr/bin/env python3
"""움직이는 뚜껑 투입(moving_drop.py)을 버튼으로 실행하는 창(실제 로봇 지령, 입회 필수).

  tools/robot/drop_ui.sh            # 환경 설정까지 해서 띄운다
  python3 tools/robot/drop_ui.py --config config/site.yaml

버튼: 한 개 옮기기 / 전부 옮기기 / 원위치로 복귀 / 중지.
창은 moving_drop.py를 하위 프로세스로 띄우고 그 JSON 줄 기록을 읽어 진행 상황을 보여 준다. 로봇 지령은 모두
moving_drop.py가 낸다. 중지 버튼은 움직이는 팔을 세우지 않는다: 뚜껑으로 들어가는 중이면 그 돌까지 놓고, 기다리는
중이면 쥔 돌을 집었던 매트 자리에 내려놓은 뒤 park로 돌아가게 한다(지금 하는 이동은 끝까지 간다).
즉시 정지는 비상정지·펜던트로 한다.
"""
from __future__ import annotations

import argparse
import json
import os
import queue
import subprocess
import sys
import threading
import time

HERE = os.path.dirname(os.path.abspath(__file__))
REPO = os.path.dirname(os.path.dirname(HERE))

ABORT_TEXT = {
    'NO_MAT_STONE': '매트에 공깃돌이 없습니다(손이 매트 위에 있으면 검출되지 않습니다)',
    'STONE_OUT_OF_REACH': '남은 공깃돌이 팔이 닿지 않는 곳에 있습니다. 로봇 쪽(x 0.69 m 안쪽)으로 옮겨 주세요',
    'NOT_RELEASED': '뚜껑이 보이지 않아 놓지 못했습니다. 벨트가 도는지 확인하세요',
    'NOT_STANDBY_AUTONOMOUS': '로봇이 대기(자동 모드) 상태가 아닙니다',
    'SERVICE_UNAVAILABLE': '로봇 드라이버 응답이 없습니다. 드라이버가 떠 있는지 확인하세요',
    'NO_STONE_IN_GRIP': '공깃돌을 집지 못했습니다(3회)',
    'NOT_HOLDING': '쥐고 있는 공깃돌이 없습니다. 다시 누르면 매트에서 집습니다',
    'MOVE_NOT_REACHED': '팔이 목표 위치에 닿지 못했습니다. 원위치로 복귀를 누르세요',
    'POSE_READ_FAILED': '로봇 위치를 읽지 못했습니다. 원위치로 복귀를 누르세요',
    'IK_OR_WORKSPACE': '닿을 수 없는 위치입니다(공깃돌 또는 대기 위치)',
    'RELEASE_POSE': '해제 위치가 작업 범위를 벗어났습니다',
    'GRIPPER_WRITE_FAILED': '그리퍼 지령을 보내지 못했습니다',
    'DRL_BUSY': '제어기에서 다른 프로그램이 돌고 있어 그리퍼 루프를 띄울 수 없습니다',
    'RESIDENT_INIT': '그리퍼 루프가 시작되지 않았습니다(그리퍼 응답 확인)',
    'RESIDENT_INIT_TIMEOUT': '그리퍼 루프 시작 시간 초과',
    'STOP_REQUESTED': '중지 요청으로 끝냈습니다',
}
STEP_TEXT = {
    'over_stone': '공깃돌 위로 이동', 'descend_grasp': '내려가서 집는 중', 'lift_stone': '공깃돌을 들어 올림',
    'lift_empty': '빈손으로 올라옴', 'wait': '벨트 옆에서 뚜껑을 기다리는 중', 'to_release': '해제 위치 도착',
    'retreat': '위로 올라옴', 'side': '벨트 옆으로 비킴', 'side_again': '다음 바퀴를 기다림',
    'park': '원위치 도착', 'park_holding': '돌을 쥔 채 원위치 도착',
    'place_back_lower': '쥔 돌을 매트에 내려놓는 중', 'place_back_lift': '돌을 내려놓고 올라옴',
}
RESULT_TEXT = {'SUCCESS': '성공', 'MISS': '실패', 'UNKNOWN': '미확인'}


def describe(event: dict) -> str | None:
    """moving_drop.py의 기록 한 줄을 화면에 보일 한국어 한 줄로 바꾼다. 보일 것이 없으면 None."""
    kind = event.get('event')
    if kind == 'ready':
        return '로봇 상태 확인'
    if kind == 'stone':
        x, y = event['pos_m'][0], event['pos_m'][1]
        return f'공깃돌 찾음 (x {x * 1000:.0f}, y {y * 1000:.0f} mm)'
    if kind == 'unreachable':
        return f"팔이 닿지 않는 공깃돌 {len(event.get('pos_m', []))}개는 건너뜁니다"
    if kind == 'loop_started':
        return '그리퍼 루프 시작'
    if kind == 'done':
        return STEP_TEXT.get(event.get('step'))
    if kind == 'gripper' and event.get('step') == 'close_grasp':
        return f"집음 ({event.get('pulse')} pulse)"
    if kind == 'grip_empty':
        return f"집지 못함 ({event.get('count')}회째). 다시 찾습니다"
    if kind == 'plan':
        return f"뚜껑 보임: {event['cap_v_mps'][0] * 1000:.0f} mm/s, {event['release_in_s']:.1f}초 뒤 도착"
    if kind == 'go':
        return '해제 위치로 들어감'
    if kind == 'fired':
        return '놓음'
    if kind == 'skip':
        return '이번 통과는 늦어서 보냄'
    if kind == 'released':
        return f"{event.get('index')}번째 돌을 놓았습니다"
    if kind == 'verdict':
        return f"{event.get('index')}번째 판정: {RESULT_TEXT.get(event.get('result'), event.get('result'))}"
    if kind == 'no_plan':
        return '뚜껑이 보이지 않습니다'
    if kind == 'not_reached':
        return f"{STEP_TEXT.get(event.get('step'), event.get('step'))}: 몇 mm 못 미침"
    if kind == 'stop_seen':
        return '중지 요청 확인'
    if kind == 'abort':
        return '중단: ' + ABORT_TEXT.get(event.get('reason'), str(event.get('reason')))
    if kind == 'summary':
        if event.get('parked'):
            return '원위치로 돌아왔습니다'
        left = event.get('unreachable') or 0
        return (f"끝: {event.get('moved', 0)}개 놓음" + (f' (닿지 않는 돌 {left}개 남음)' if left else '')
                + (' · 돌을 쥔 채 원위치' if event.get('holding') else ''))
    return None


def build_command(mode: str, config: str, speed: float = 1.0, record: str | None = None,
                  stop_file: str | None = None, holding: bool = False, script: str | None = None,
                  drop_delay_s: float | None = None) -> list[str]:
    """mode: 'one' | 'all' | 'park'. moving_drop.py 실행 명령을 만든다."""
    if mode not in ('one', 'all', 'park'):
        raise ValueError(mode)
    cmd = [sys.executable, script or os.path.join(HERE, 'moving_drop.py'), '--config', config, '--execute']
    if mode == 'park':
        return cmd + ['--park-only']
    cmd += ['--count', '1' if mode == 'one' else '0', '--speed-scale', f'{speed:g}']
    if drop_delay_s is not None:
        cmd += ['--drop-delay-s', f'{drop_delay_s:g}']
    if holding:
        cmd.append('--holding')
    if stop_file:
        cmd += ['--stop-file', stop_file]
    if record:
        cmd += ['--record', record]
    return cmd


class Tally:
    """기록 줄에서 놓은 수와 판정 수를 센다."""

    def __init__(self):
        self.moved = self.success = self.miss = self.unknown = 0
        self.last_reason = None
        self.holding = None             # 끝날 때 돌을 쥐고 있는지(summary가 알려 준다)

    def feed(self, event: dict) -> None:
        kind = event.get('event')
        if kind == 'released':
            self.moved += 1
        elif kind == 'verdict':
            result = event.get('result')
            self.success += result == 'SUCCESS'
            self.miss += result == 'MISS'
            self.unknown += result not in ('SUCCESS', 'MISS')
        if kind in ('abort', 'summary') and event.get('reason'):
            self.last_reason = event['reason']
        if kind == 'summary' and event.get('holding') is not None:
            self.holding = bool(event['holding'])

    def text(self) -> str:
        return f'놓음 {self.moved} · 성공 {self.success} · 실패 {self.miss} · 미확인 {self.unknown}'


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description=__doc__.split('\n')[0])
    ap.add_argument('--config', default=os.path.join(REPO, 'config/site.yaml'))
    ap.add_argument('--record-dir', default=os.path.join(REPO, 'logs', time.strftime('%Y-%m-%d')),
                    help='실행 기록·녹화 폴더(저장소의 logs/는 git이 무시한다)')
    ap.add_argument('--script', help='moving_drop.py 대신 실행할 스크립트(시험용)')
    ap.add_argument('--self-test', action='store_true', help='창을 만들었다가 바로 닫는다(시험용)')
    ap.add_argument('--auto', choices=['one', 'all', 'park'], help='그 버튼을 한 번 누르고 끝나면 창을 닫는다(시험용)')
    ap.add_argument('--speed', type=float, choices=[1.0, 1.5, 2.0], default=1.0, help='속도 선택의 처음 값')
    ap.add_argument('--no-record', action='store_true', help='녹화 선택을 끈 채로 시작한다')
    ap.add_argument('--stop-after-s', type=float, help='--auto 실행 중 이 시간 뒤 중지 버튼을 누른다(시험용)')
    args = ap.parse_args(argv)

    import tkinter as tk
    from tkinter import messagebox, scrolledtext

    root = tk.Tk()
    root.title('공깃돌 옮기기')
    root.geometry('760x560')
    root.configure(cursor='left_ptr')
    state = {'proc': None, 'tally': Tally(), 'holding': False, 'mode': None, 'log_file': None}
    lines: queue.Queue = queue.Queue()
    stop_file = os.path.join(args.record_dir, 'STOP')

    status = tk.StringVar(value='대기 중')
    counts = tk.StringVar(value=state['tally'].text())
    speed = tk.DoubleVar(value=args.speed)
    record = tk.BooleanVar(value=not args.no_record)

    tk.Label(root, textvariable=status, font=('', 20, 'bold'), anchor='w', wraplength=720, justify='left').pack(
        fill='x', padx=16, pady=(14, 2))
    tk.Label(root, textvariable=counts, font=('', 13), anchor='w').pack(fill='x', padx=16, pady=(0, 10))

    row = tk.Frame(root)
    row.pack(fill='x', padx=12)
    buttons = {}

    def add_button(key, text, command, **kw):
        b = tk.Button(row, text=text, command=command, font=('', 14), height=2, **kw)
        b.pack(side='left', expand=True, fill='x', padx=4)
        buttons[key] = b

    opts = tk.Frame(root)
    opts.pack(fill='x', padx=16, pady=(10, 4))
    tk.Label(opts, text='집기·이동 속도:').pack(side='left')
    for label, value in (('보통', 1.0), ('1.5배', 1.5), ('2배', 2.0)):
        tk.Radiobutton(opts, text=label, variable=speed, value=value).pack(side='left')
    tk.Checkbutton(opts, text='영상·검출 녹화', variable=record).pack(side='left', padx=(24, 0))

    box = scrolledtext.ScrolledText(root, height=14, font=('', 11), state='disabled')
    box.pack(fill='both', expand=True, padx=16, pady=(6, 4))
    tk.Label(root, text='즉시 정지는 비상정지·펜던트. 실행 중에는 매트·벨트 위로 손을 넣지 마세요(돌이 검출되지 않습니다).',
             fg='#a03000', anchor='w').pack(fill='x', padx=16, pady=(0, 10))

    def say(text):
        box.configure(state='normal')
        box.insert('end', time.strftime('%H:%M:%S ') + text + '\n')
        box.see('end')
        box.configure(state='disabled')

    def set_running(running):
        for key in ('one', 'all', 'park'):
            buttons[key].configure(state='disabled' if running else 'normal')
        buttons['stop'].configure(state='normal' if running and state['mode'] != 'park' else 'disabled')

    def reader(proc, log_path):
        log = open(log_path, 'w') if log_path else None
        for line in proc.stdout:
            if log:
                log.write(line)
                log.flush()
            lines.put(line)
        proc.wait()
        if log:
            log.close()
        lines.put(None)

    def start(mode):
        if state['proc'] is not None:
            return
        os.makedirs(args.record_dir, exist_ok=True)
        if os.path.exists(stop_file):
            os.remove(stop_file)
        prefix = os.path.join(args.record_dir, 'ui-' + time.strftime('%H%M%S'))
        cmd = build_command(mode, args.config, speed.get(), prefix if (record.get() and mode != 'park') else None,
                            stop_file, state['holding'] and mode != 'park', args.script)
        state.update(mode=mode, tally=Tally())
        counts.set(state['tally'].text())
        status.set({'one': '한 개 옮기는 중', 'all': '전부 옮기는 중', 'park': '원위치로 복귀 중'}[mode])
        say('── ' + status.get() + (' (쥐고 있는 돌부터)' if '--holding' in cmd else ''))
        env = dict(os.environ, PYTHONPATH=REPO + os.pathsep + os.environ.get('PYTHONPATH', ''), PYTHONUNBUFFERED='1')
        proc = subprocess.Popen(cmd, stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True, cwd=REPO, env=env)
        state['proc'] = proc
        set_running(True)
        threading.Thread(target=reader, args=(proc, prefix + '.jsonl' if mode != 'park' else None), daemon=True).start()

    def stop():
        if state['proc'] is None:
            return
        open(stop_file, 'w').close()
        status.set('중지 요청: 쥔 돌은 놓거나 매트에 내려놓고 원위치로 갑니다')
        say('중지 요청')
        buttons['stop'].configure(state='disabled')

    def finished():
        proc, tally = state['proc'], state['tally']
        state['proc'] = None
        code = proc.returncode
        if state['mode'] != 'park':
            state['holding'] = tally.holding if tally.holding is not None else tally.last_reason == 'NOT_RELEASED'
        if code == 0:
            head = '중지했습니다' if tally.last_reason == 'STOP_REQUESTED' else '끝났습니다'
            status.set('원위치로 돌아왔습니다' if state['mode'] == 'park' else head + ' · ' + tally.text())
        else:
            status.set('중단: ' + ABORT_TEXT.get(tally.last_reason, str(tally.last_reason or f'종료 코드 {code}')))
        set_running(False)
        if args.auto:
            print(json.dumps({'event': 'auto_done', 'status': status.get(), 'counts': counts.get(), 'code': code,
                              'holding': state['holding']}, ensure_ascii=False))
            root.after(200, root.destroy)

    def poll():
        try:
            while True:
                line = lines.get_nowait()
                if line is None:
                    finished()
                    continue
                if not line.startswith('{'):
                    continue
                try:
                    event = json.loads(line)
                except ValueError:
                    continue
                state['tally'].feed(event)
                counts.set(state['tally'].text())
                text = describe(event)
                if text:
                    say(text)
                    if event.get('event') in ('done', 'plan', 'go', 'fired', 'stone') and state['proc'] is not None:
                        status.set(text)
        except queue.Empty:
            pass
        root.after(100, poll)

    def on_close():
        if state['proc'] is not None:
            messagebox.showwarning('실행 중', '로봇이 움직이는 중입니다. 중지를 누르고 원위치로 돌아온 뒤 닫으세요.\n'
                                              '즉시 정지는 비상정지·펜던트로 합니다.')
            return
        root.destroy()

    add_button('one', '한 개 옮기기', lambda: start('one'))
    add_button('all', '전부 옮기기', lambda: start('all'))
    add_button('park', '원위치로 복귀', lambda: start('park'))
    add_button('stop', '중지\n(돌 내려놓고 원위치)', stop, fg='#a03000')
    set_running(False)
    root.protocol('WM_DELETE_WINDOW', on_close)
    root.after(100, poll)
    if args.self_test:
        root.update()
        print(json.dumps({'event': 'self_test', 'buttons': sorted(buttons), 'size': [root.winfo_width(), root.winfo_height()]}))
        root.destroy()
        return 0
    def click(key):
        """버튼을 마우스로 누른 것처럼 포인터를 옮겨 누르고 뗀다(--auto 시연·시험용)."""
        b = buttons[key]
        x, y = b.winfo_width() // 2, b.winfo_height() // 2
        b.event_generate('<Motion>', warp=True, x=x, y=y)
        b.event_generate('<Enter>', x=x, y=y)
        root.after(250, lambda: b.event_generate('<ButtonPress-1>', x=x, y=y))
        root.after(450, lambda: b.event_generate('<ButtonRelease-1>', x=x, y=y))

    if args.auto:
        root.after(800, lambda: click(args.auto))
        if args.stop_after_s:
            root.after(1300 + int(args.stop_after_s * 1000), lambda: click('stop'))
    root.mainloop()
    return 0


if __name__ == '__main__':
    sys.exit(main())
