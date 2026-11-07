"""record_logger_node 시험(PLAN.md 10.1절)."""
import json
import os

import pytest
from builtin_interfaces.msg import Time

rclpy = pytest.importorskip('rclpy')

from orchestrator.record_logger_node import (RecordLoggerNode, step_record_to_dict,  # noqa: E402
                                             steps_path)
from drop_interfaces.msg import StepRecord  # noqa: E402
from rclpy.parameter import Parameter  # noqa: E402

from drop_ros import DOMAIN, Harness  # noqa: E402


def _time(sec):
    ns = int(round(sec * 1e9))
    return Time(sec=ns // 1_000_000_000, nanosec=ns % 1_000_000_000)


def _record(cycle_id, seq, name='approach', t0=10.5, t1=12.25, result=0,
            reason='', attempt=1, abort=0, keys=('e_xy_m',), values=(0.012,)):
    msg = StepRecord()
    msg.header.stamp = _time(9.75)
    msg.cycle_id = cycle_id
    msg.step_seq = seq
    msg.step_name = name
    msg.t_start = _time(t0)
    msg.t_end = _time(t1)
    msg.result = result
    msg.reason_code = reason
    msg.attempt = attempt
    msg.abort_level = abort
    msg.metric_keys = list(keys)
    msg.metric_values = list(values)
    return msg


@pytest.mark.level('U')
def test_step_record_to_dict_fields_and_metrics():
    msg = _record('c1', 3, keys=('e_xy_m', 'v_rel_mps'), values=(0.25, -0.5))
    d = step_record_to_dict(msg)
    assert d == {
        'stamp_s': pytest.approx(9.75),
        'cycle_id': 'c1',
        'step_seq': 3,
        'step_name': 'approach',
        't_start_s': pytest.approx(10.5),
        't_end_s': pytest.approx(12.25),
        'result': 0,
        'reason_code': '',
        'attempt': 1,
        'abort_level': 0,
        'metric_keys': ['e_xy_m', 'v_rel_mps'],
        'metric_values': [0.25, -0.5],
        'metrics': {'e_xy_m': 0.25, 'v_rel_mps': -0.5},
    }
    assert d['stamp_s'] == 9.75
    assert json.loads(json.dumps(d, ensure_ascii=False)) == d


@pytest.mark.level('U')
def test_step_record_to_dict_empty_metrics():
    d = step_record_to_dict(_record('c1', 0, keys=(), values=()))
    assert d['metric_keys'] == [] and d['metric_values'] == [] and d['metrics'] == {}


@pytest.mark.level('U')
def test_steps_path_normal_and_empty():
    assert steps_path('/r', 'cyc_1', '2026-11-07') == '/r/2026-11-07/cyc_1.steps.jsonl'
    assert steps_path('/r', '', '2026-11-07') == '/r/2026-11-07/no_cycle.steps.jsonl'


@pytest.mark.level('U')
@pytest.mark.parametrize('bad', ['../x', 'a/b', '..', 'a..b'])
def test_steps_path_rejects_escape(bad):
    with pytest.raises(ValueError):
        steps_path('/r', bad, '2026-11-07')


@pytest.fixture(scope='module')
def ros():
    rclpy.init(domain_id=DOMAIN)
    yield
    rclpy.shutdown()


@pytest.mark.level('C')
def test_node_writes_jsonl_per_cycle(ros, tmp_path):
    date = '2026-11-07'
    logger = RecordLoggerNode(parameter_overrides=[
        Parameter('records_dir', value=str(tmp_path)),
        Parameter('date', value=date),
    ])
    pub_node = rclpy.create_node('test_step_record_pub')
    h = Harness(logger, pub_node)
    try:
        pub = pub_node.create_publisher(StepRecord, 'task/step_record', 100)
        h.wait_for(lambda: pub.get_subscription_count() > 0, what='subscription')
        msgs = [
            _record('cyc_a', 0, name='approach', keys=('e_xy_m',), values=(0.01,)),
            _record('cyc_b', 0, name='pick', result=1, reason='NO_STONE',
                    keys=('v_rel_mps',), values=(0.2,)),
            _record('cyc_a', 1, name='place', keys=('e_xy_m', 'e_yaw_rad'),
                    values=(0.02, 0.03)),
        ]
        for m in msgs:
            pub.publish(m)

        a_path = os.path.join(str(tmp_path), date, 'cyc_a.steps.jsonl')
        b_path = os.path.join(str(tmp_path), date, 'cyc_b.steps.jsonl')

        def done():
            if not (os.path.exists(a_path) and os.path.exists(b_path)):
                return False
            with open(a_path) as fa, open(b_path) as fb:
                return len(fa.readlines()) >= 2 and len(fb.readlines()) >= 1

        h.wait_for(done, timeout=5.0, what='JSONL 기록')
        assert logger.written == 3

        with open(a_path, encoding='utf-8') as fh:
            a = [json.loads(line) for line in fh]
        with open(b_path, encoding='utf-8') as fh:
            b = [json.loads(line) for line in fh]
        assert len(a) == 2 and len(b) == 1
        assert a == [step_record_to_dict(msgs[0]), step_record_to_dict(msgs[2])]
        assert b == [step_record_to_dict(msgs[1])]
    finally:
        h.close()
