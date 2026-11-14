"""Run only the companion gripper timing DRLs; never sends arm commands.

Config pages and timing results use output register bytes 15..23. Each page is
held for 2 s; marker is read twice around a snapshot to reject page transitions.
"""
import argparse
import json
import time
from pathlib import Path

import rclpy
from dsr_msgs2.srv import DrlStart, GetDrlState, GetLastAlarm, GetOutputRegisterInt, GetRobotState


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('file', type=Path)
    parser.add_argument('--timeout', type=float, default=35)
    parser.add_argument('--ack-alarm', type=int, action='append', default=[],
                        help='이미 확인한 지난 알람 index(예: 1206 도달 불가 거절). robot_state=1일 때만 통과')
    args = parser.parse_args()
    rclpy.init()
    node = rclpy.create_node('gripper_timing_run')
    clients = {}

    def call(typ, suffix, **fields):
        key = (typ, suffix)
        if key not in clients:
            client = node.create_client(typ, '/dsr01/' + suffix)
            if not client.wait_for_service(3.0):
                raise RuntimeError('service unavailable: ' + suffix)
            clients[key] = client
        future = clients[key].call_async(typ.Request(**fields))
        rclpy.spin_until_future_complete(node, future, timeout_sec=3.0)
        if not future.done() or future.result() is None:
            raise RuntimeError('service timeout: ' + suffix)
        result = future.result()
        if hasattr(result, 'success') and not result.success:
            raise RuntimeError('service unsuccessful: ' + suffix)
        return result

    def safety():
        state = call(GetRobotState, 'system/get_robot_state').robot_state
        alarm = call(GetLastAlarm, 'system/get_last_alarm').log_alarm
        info = {'robot_state': state, 'alarm_level': alarm.level,
                'alarm_group': alarm.group, 'alarm_index': alarm.index,
                'alarm_param': list(alarm.param)}
        if state != 1 or (alarm.level >= 2 and alarm.index not in args.ack_alarm):
            raise RuntimeError('STOP, report to the operator: ' + json.dumps(info))
        return info

    def byte(addr):
        return (call(GetOutputRegisterInt, 'plc/get_output_register_int',
                     address=addr, timeout_ms=1000).value >> 24) & 255

    try:
        print(json.dumps({'preflight': safety()}), flush=True)
        drl_state = call(GetDrlState, 'drl/get_drl_state').drl_state
        # 1 = STOP, 3 = LAST(드라이버 연결 뒤 아직 실행한 프로그램이 없음). 0 PLAY·2 HOLD는 다른 DRL이 있다
        if drl_state not in (1, 3):
            raise RuntimeError('DRL is not stopped: ' + str(drl_state))
        call(DrlStart, 'drl/drl_start', robot_system=0, code=args.file.read_text())
        print(json.dumps({'drl_start': True, 'file': str(args.file)}), flush=True)
        deadline = time.monotonic() + args.timeout
        pages = {}
        time.sleep(0.3)
        while time.monotonic() < deadline:
            safety()
            marker = byte(15)
            if marker >= 240 and marker != 250:
                raise RuntimeError('DRL failure marker: ' + str(marker))
            if marker in (1, 2, 3, 4) and marker not in pages:
                payload = [byte(a) for a in range(16, 24)]
                if byte(15) == marker:
                    pages[marker] = payload
                    print(json.dumps({'page': marker, 'bytes': payload}), flush=True)
            if marker == 250:
                print(json.dumps({'finished': True, 'postflight': safety()}), flush=True)
                return
            state = call(GetDrlState, 'drl/get_drl_state').drl_state
            if state == 1 and marker not in (0, 250):
                raise RuntimeError('DRL stopped without completion: ' + str(marker))
            time.sleep(0.05)
        raise RuntimeError('DRL timeout; inspect before considering drl_stop')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
