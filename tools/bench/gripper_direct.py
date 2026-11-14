#!/usr/bin/env python3
"""두산 E0509 RH-P12-RN 그리퍼 제어 스크립트.

하드웨어/참고 구현 범위(0~750 pulse):
  - open  : 0 pulse (완전 열림, 핑거 최대 간격)
  - close : 750 pulse (완전 닫힘)
"""

import argparse
import socket
import struct

ROBOT_IP = "110.120.1.38"
MODBUS_PORT = 502

REG_SEQ = 10
REG_POS_HI = 11
REG_POS_LO = 12
REG_CUR_HI = 13
REG_CUR_LO = 14

PULSE_OPEN = 0     # 완전 열림
PULSE_CLOSE = 750  # 참고 구현 및 상주 DRL 상한 (완전 닫힘)

def encode7(value: int) -> tuple[int, int]:
    return (value >> 7) & 0x7F, value & 0x7F

def write_holding_register(sock: socket.socket, addr: int, val: int):
    msg = struct.pack('>HHHBBHH', 1, 0, 6, 1, 6, addr, val)
    sock.sendall(msg)
    res = sock.recv(1024)
    if len(res) < 12:
        raise RuntimeError(f"Modbus write failed at addr {addr}")
    return res

def read_holding_register(sock: socket.socket, addr: int) -> int:
    msg = struct.pack('>HHHBBHH', 1, 0, 6, 1, 3, addr, 1)
    sock.sendall(msg)
    res = sock.recv(1024)
    if len(res) < 11:
        raise RuntimeError(f"Modbus read failed at addr {addr}")
    val = struct.unpack('>H', res[9:11])[0]
    return val

def send_gripper_command(pulse: int, current_ma: int = 100):
    if type(current_ma) is not int or not 1 <= current_ma <= 100:
        raise ValueError("OUT_OF_RANGE current: allowed 1..100 mA")
    ph, pl = encode7(pulse)
    ch, cl = encode7(current_ma)
    
    sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    sock.settimeout(3.0)
    sock.connect((ROBOT_IP, MODBUS_PORT))
    
    prev_seq = read_holding_register(sock, REG_SEQ)
    next_seq = (prev_seq % 127) + 1
    
    write_holding_register(sock, REG_POS_HI, ph)
    write_holding_register(sock, REG_POS_LO, pl)
    write_holding_register(sock, REG_CUR_HI, ch)
    write_holding_register(sock, REG_CUR_LO, cl)
    write_holding_register(sock, REG_SEQ, next_seq)
    
    sock.close()
    return next_seq

def main():
    parser = argparse.ArgumentParser(description="두산 E0509 그리퍼 직접 제어")
    parser.add_argument("action", choices=["open", "close", "set"], help="동작")
    parser.add_argument("--pulse", type=int, default=PULSE_CLOSE, help="목표 pulse (0~750)")
    parser.add_argument("--current", type=int, default=100, help="전류 [mA] (1~100)")
    args = parser.parse_args()
    if not 1 <= args.current <= 100:
        parser.error("--current 는 1~100 mA 범위여야 합니다.")

    if args.action == "open":
        target_pulse = PULSE_OPEN
        print(f"[그리퍼 열기] pulse={target_pulse}, current={args.current}mA")
    elif args.action == "close":
        target_pulse = PULSE_CLOSE
        print(f"[그리퍼 닫기] pulse={target_pulse}, current={args.current}mA")
    else:
        target_pulse = min(PULSE_CLOSE, max(0, args.pulse))
        print(f"[그리퍼 이동] pulse={target_pulse}, current={args.current}mA")

    seq = send_gripper_command(target_pulse, args.current)
    print(f"-> 전송 완료 (seq: {seq})")

if __name__ == "__main__":
    main()
