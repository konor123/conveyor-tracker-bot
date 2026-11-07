"""record_logger_node 시험(PLAN.md 10.1절)."""
import json
import os

import pytest
from builtin_interfaces.msg import Time

rclpy = pytest.importorskip('rclpy')

from orchestrator.record_logger_node import (RecordLoggerNode, build_trial_record,  # noqa: E402
                                             step_record_to_dict, steps_path,
                                             trials_path, validate_trial_record)
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
    assert trials_path('/r', '2026-11-07') == '/r/2026-11-07/trials.jsonl'


@pytest.mark.level('U')
@pytest.mark.parametrize('bad', ['../x', 'a/b', '..', 'a..b'])
def test_steps_path_rejects_escape(bad):
    with pytest.raises(ValueError):
        steps_path('/r', bad, '2026-11-07')


@pytest.mark.req('REQ-REC-001', 'REQ-REC-002')
@pytest.mark.tc('TC-REC-002-01')
@pytest.mark.level('U')
def test_build_and_validate_trial_record():
    rec = build_trial_record(
        {
            'trial_id': '20261107-0001',
            'mode': 'MODE_SYNCHRONIZED',
            'result': 'NOT_ATTEMPTED',
            'abort_level': 3,
            'reason_code': 'PASS_SKIP',
            'passes_skipped': 3,
            'cycle_time_s': 9.5,
            'step_durations_s': {'WAIT_CAP': 3.5},
        },
        env='mock',
        gate='M0',
        git_sha='abc1234',
        scenario_name='time_insufficient',
        seed=0,
        assumed_params=['world.belt.speed_mps', 'robot.skills.PickStone.duration_s'],
    )
    assert validate_trial_record(rec) == []
    assert rec['result'] == 'NOT_ATTEMPTED'
    assert rec['passes_skipped'] == 3
    assert rec['assumed_params'] == ['world.belt.speed_mps', 'robot.skills.PickStone.duration_s']


@pytest.mark.req('REQ-REC-001')
@pytest.mark.tc('TC-REC-001-01')
@pytest.mark.level('U')
def test_schema_validation_from_external_symlink_build(tmp_path, monkeypatch):
    import orchestrator.record_logger_node as logger_module

    project = tmp_path / 'workspace'
    source = project / 'src' / 'orchestrator' / 'orchestrator'
    source.mkdir(parents=True)
    schema = project / 'tools' / 'eval' / 'trial_schema.json'
    schema.parent.mkdir(parents=True)
    schema.write_text(json.dumps({'type': 'object', 'required': ['trial_id']}))
    build = tmp_path / 'external-build' / 'orchestrator'
    build.mkdir(parents=True)
    (build / 'orchestrator').symlink_to(source, target_is_directory=True)
    monkeypatch.setattr(logger_module, '__file__', str(build / 'orchestrator' / 'record_logger_node.py'))
    monkeypatch.chdir(tmp_path)

    assert logger_module.validate_trial_record({'trial_id': 'valid'}) == []
    assert logger_module.validate_trial_record({})


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
    pub_node = rclpy.create_node('orchestrator_node')
    pub_node.declare_parameter('last_trial_json', '')
    mock_node = rclpy.create_node('mock_world_node')
    mock_node.declare_parameter('scenario', 'nominal_20mms')
    mock_node.declare_parameter('seed', 7)
    mock_node.declare_parameter('assumed_params', ['world.belt.speed_mps', 'robot.duration_scale_ratio'])
    h = Harness(logger, pub_node, mock_node)
    try:
        pub = pub_node.create_publisher(StepRecord, 'task/step_record', 100)
        h.wait_for(lambda: pub.get_subscription_count() > 0, what='subscription')
        h.wait_for(lambda: logger.assumed_params == ['world.belt.speed_mps', 'robot.duration_scale_ratio'],
                   timeout=5.0, what='assumed_params 동기화')
        msgs = [
            _record('cyc_a', 0, name='approach', keys=('e_xy_m',), values=(0.01,)),
            _record('cyc_b', 0, name='pick', result=1, reason='NO_STONE',
                    keys=('v_rel_mps',), values=(0.2,)),
            _record('cyc_a', 1, name='place', keys=('e_xy_m', 'e_yaw_rad'),
                    values=(0.02, 0.03)),
        ]
        for m in msgs:
            pub.publish(m)

        pub_node.set_parameters([Parameter('last_trial_json', value=json.dumps({
            'trial_id': 'cyc_a',
            'mode': 'MODE_SYNCHRONIZED',
            'result': 'SUCCESS',
            'abort_level': 0,
            'reason_code': '',
            'passes_skipped': 0,
            'cycle_time_s': 9.1,
            'step_durations_s': {'APPROACH': 0.6},
        }))])

        a_path = os.path.join(str(tmp_path), date, 'cyc_a.steps.jsonl')
        b_path = os.path.join(str(tmp_path), date, 'cyc_b.steps.jsonl')
        t_path = os.path.join(str(tmp_path), date, 'trials.jsonl')

        def done():
            if not (os.path.exists(a_path) and os.path.exists(b_path) and os.path.exists(t_path)):
                return False
            with open(a_path) as fa, open(b_path) as fb, open(t_path) as ft:
                return len(fa.readlines()) >= 2 and len(fb.readlines()) >= 1 and len(ft.readlines()) >= 1

        h.wait_for(done, timeout=5.0, what='JSONL 기록')
        assert logger.written == 3
        assert logger.trials_written == 1

        with open(a_path, encoding='utf-8') as fh:
            a = [json.loads(line) for line in fh]
        with open(b_path, encoding='utf-8') as fh:
            b = [json.loads(line) for line in fh]
        with open(t_path, encoding='utf-8') as fh:
            trials = [json.loads(line) for line in fh]
        assert len(a) == 2 and len(b) == 1 and len(trials) == 1
        assert a == [step_record_to_dict(msgs[0]), step_record_to_dict(msgs[2])]
        assert b == [step_record_to_dict(msgs[1])]
        assert validate_trial_record(trials[0]) == []
        assert trials[0]['trial_id'] == 'cyc_a'
        assert trials[0]['scenario'] == {'name': 'nominal_20mms', 'seed': 7}
        assert trials[0]['assumed_params'] == ['world.belt.speed_mps', 'robot.duration_scale_ratio']
    finally:
        h.close()
