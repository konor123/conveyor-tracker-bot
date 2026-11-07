"""record_logger_node(PLAN.md 10.1절): StepRecord → records/trials/<date>/<cycle_id>.steps.jsonl."""
from __future__ import annotations

import datetime
import json
import os

import rclpy
from drop_interfaces.msg import StepRecord
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy


def _time_to_s(t) -> float:
    return t.sec + t.nanosec * 1e-9


def step_record_to_dict(msg: StepRecord) -> dict:
    keys = list(msg.metric_keys)
    values = list(msg.metric_values)
    return {
        'stamp_s': _time_to_s(msg.header.stamp),
        'cycle_id': msg.cycle_id,
        'step_seq': int(msg.step_seq),
        'step_name': msg.step_name,
        't_start_s': _time_to_s(msg.t_start),
        't_end_s': _time_to_s(msg.t_end),
        'result': int(msg.result),
        'reason_code': msg.reason_code,
        'attempt': int(msg.attempt),
        'abort_level': int(msg.abort_level),
        'metric_keys': keys,
        'metric_values': values,
        'metrics': dict(zip(keys, values)),
    }


def steps_path(root: str, cycle_id: str, date: str) -> str:
    if not cycle_id:
        cycle_id = 'no_cycle'
    if '/' in cycle_id or '..' in cycle_id:
        raise ValueError(f'잘못된 cycle_id: {cycle_id!r}')
    return os.path.join(root, date, f'{cycle_id}.steps.jsonl')


class RecordLoggerNode(Node):
    def __init__(self, **kwargs):
        super().__init__('record_logger_node', **kwargs)
        default_dir = os.environ.get('DROP_RECORDS_DIR') or os.path.join(
            os.getcwd(), 'records', 'trials')
        self.declare_parameter('records_dir', default_dir)
        self.declare_parameter('date', '')
        self._root = str(self.get_parameter('records_dir').value)
        date = str(self.get_parameter('date').value)
        self._date = date or datetime.date.today().isoformat()
        self.written = 0
        qos = QoSProfile(depth=100, reliability=ReliabilityPolicy.RELIABLE,
                         durability=DurabilityPolicy.VOLATILE)
        self.create_subscription(StepRecord, 'task/step_record', self._on_step_record, qos)

    def _on_step_record(self, msg: StepRecord) -> None:
        try:
            path = steps_path(self._root, msg.cycle_id, self._date)
        except ValueError as e:              # 기록 하나 때문에 노드가 죽지 않게 한다
            self.get_logger().warning(f'StepRecord 건너뜀: {e}')
            return
        os.makedirs(os.path.dirname(path), exist_ok=True)
        line = json.dumps(step_record_to_dict(msg), ensure_ascii=False)
        with open(path, 'a', encoding='utf-8') as fh:
            fh.write(line + '\n')
            fh.flush()
        self.written += 1


def main(args=None):
    rclpy.init(args=args)
    node = RecordLoggerNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
