"""ROS-free execution of production node classes with transport stubs."""
import ast
import json
import os
import threading
import time
from functools import wraps
from pathlib import Path
from types import SimpleNamespace as NS

import pytest

from orchestrator.abort_policy import Event, HARD_FAULTS
from orchestrator.common import Mode

ROOT = Path(__file__).parents[1] / 'orchestrator'


def load(name, symbols, **extra):
    tree = ast.parse((ROOT / name).read_text())
    tree.body = [n for n in tree.body if isinstance(n, (ast.ClassDef, ast.FunctionDef)) and n.name in symbols
                 or isinstance(n, ast.ImportFrom) and n.module == '__future__']
    env = dict(Node=object, threading=threading, time=time, os=os, json=json,
               wraps=wraps, Mode=Mode, Event=Event, HARD_FAULTS=HARD_FAULTS, POLL_S=.001,
               _to_time=lambda x: x, rclpy=NS(ok=lambda **kw: True), **extra)
    exec(compile(tree, str(ROOT / name), 'exec'), env)
    return env


def port():
    cls = load('orchestrator_node.py', ['RosSkillPort'])['RosSkillPort']
    n = NS(clock=NS(ros_now=lambda: 0.), cycle_id='c', step_id=1,
           cfg_dict={'watchdog': {'cycle_steady_factor_ratio': 2.0}},
           get_logger=lambda: NS(warning=lambda x: None), _watch=lambda: None)
    p = cls(n)
    p.reset(NS(is_cancel_requested=False))
    return p


@pytest.mark.req('REQ-ABT-004')
@pytest.mark.tc('TC-ABT-004-02')
@pytest.mark.level('U')
def test_pending_acceptance_is_not_cancel_ack_and_late_handle_is_cancelled():
    p = port()
    ready = [False]
    cancelled = []
    result = NS(done=lambda: True, result=lambda: NS(status=5))
    gh = NS(accepted=True, get_result_async=lambda: result,
            cancel_goal_async=lambda: cancelled.append(True))
    fut = NS(done=lambda: ready[0], result=lambda: gh)
    p.n.actions = {'pick': NS(server_is_ready=lambda: True, send_goal_async=lambda *a, **kw: fut)}
    p._wait = lambda pred, deadline: pred()
    assert p._send('pick', NS(), 1.) is None
    assert not p.cancel_active(.002)
    ready[0] = True
    assert p.cancel_active(.02)
    assert cancelled == [True]


@pytest.mark.req('REQ-ABT-003')
@pytest.mark.tc('TC-ABT-003-03')
@pytest.mark.level('U')
def test_wait_cancel_and_steady_guard():
    p = port()
    p.drop_goal_handle.is_cancel_requested = True
    assert not p._wait(lambda: False, 0.)
    assert p.pending_fault() is Event.USER_CANCEL
    p.reset(NS(is_cancel_requested=False))
    p.set_wait_guard(lambda: Event.CYCLE_TIMEOUT)
    assert not p._wait(lambda: False, 1.)
    assert p.pending_fault() is Event.CYCLE_TIMEOUT


@pytest.mark.req('REQ-REC-002')
@pytest.mark.tc('TC-REC-002-02')
@pytest.mark.level('U')
@pytest.mark.parametrize('bad', ['[]', '{"trial_id":"x","cycle_time_s":"bad"}', None])
def test_logger_invalid_root_and_io_recovery(tmp_path, bad):
    env = load('record_logger_node.py', ['RecordLoggerNode', '_serialized', 'build_trial_record',
               '_clean_scenario_name', 'trials_path'], validate_trial_record=lambda rec: [])
    cls = env['RecordLoggerNode']
    n = cls.__new__(cls)
    n._io_lock = threading.RLock()
    n._root, n._date = str(tmp_path / 'blocked'), '2026-11-09'
    (tmp_path / 'blocked').write_text('file')
    n._env, n._gate, n._git_sha, n._scenario_name, n._seed = 'mock', 'M0', 'sha', 'nominal', 0
    n.assumed_params, n._last_trial_id, n.trials_written = ['x'], '', 0
    n.get_logger = lambda: NS(warning=lambda msg: None)
    if bad is not None:
        n._write_trial_from_json(bad)
    n._write_trial_from_json('{"trial_id":"x"}')
    assert n._last_trial_id == '' and n.trials_written == 0
    (tmp_path / 'blocked').unlink()
    n._write_trial_from_json('{"trial_id":"x"}')
    assert n._last_trial_id == 'x' and n.trials_written == 1


@pytest.mark.req('REQ-REC-002')
@pytest.mark.tc('TC-REC-002-02')
@pytest.mark.level('U')
def test_missing_schema_fails_closed(monkeypatch):
    env = load('record_logger_node.py', ['validate_trial_record'], _find_schema_path=lambda: None)
    assert env['validate_trial_record']({})


@pytest.mark.req('REQ-ABT-009')
@pytest.mark.tc('TC-ABT-009-03')
@pytest.mark.level('U')
def test_supervisor_retries_until_confirmed_with_one_inflight():
    stop = NS(Request=lambda **kw: NS(**kw))
    stop.Request.SUPERVISOR = 1
    env = load('safety_supervisor_node.py', ['SafetySupervisorNode'], Stop=stop)
    n = env['SafetySupervisorNode'].__new__(env['SafetySupervisorNode'])
    n.monitor = NS(check=lambda: None, active_fault=lambda: 'ROBOT_COMM_LOST')
    n.steady_clock = NS(steady_now=lambda: 1.)
    n._stop_future, n._stop_reason, n._stop_confirmed, n._next_stop_try = None, None, False, 0.
    n.stop_calls = []
    n.get_logger = lambda: NS(warning=lambda x: None)
    ready, done, calls = [False], [False], []
    fut = NS(done=lambda: done[0], result=lambda: NS(stopped=True))
    n._stop_cli = NS(service_is_ready=lambda: ready[0], call_async=lambda req: calls.append(req) or fut)
    n._tick()
    ready[0] = True
    n._next_stop_try = 0.
    n._tick()
    n._tick()
    assert len(calls) == 1
    done[0] = True
    n._tick()
    assert n._stop_confirmed


def bounded_wait(p, deadline):
    results = []
    worker = threading.Thread(target=lambda: results.append(p._wait(lambda: False, deadline)), daemon=True)
    worker.start()
    worker.join(.1)
    returned = not worker.is_alive()
    if not returned:
        p.raise_fault(Event.PROTECTIVE_STOP)
        worker.join(1.)
    assert returned, 'port ignored steady deadline or shutdown'
    return results[0]


@pytest.mark.req('REQ-ABT-003', 'REQ-ABT-008')
@pytest.mark.tc('TC-ABT-003-03')
@pytest.mark.level('U')
@pytest.mark.parametrize('cause', ['steady', 'shutdown', 'cleanup'])
def test_wait_frozen_clock_shutdown_and_cleanup(cause):
    p = port()
    if cause == 'shutdown':
        p.n.context = NS(ok=lambda: False)
        assert not bounded_wait(p, 1.)
        assert p.pending_fault() is Event.ROBOT_COMM_LOST
    elif cause == 'cleanup':
        p.drop_goal_handle.is_cancel_requested = True
        p.set_wait_guard(lambda: None, cleanup=True)
        assert p._wait(lambda: True, 1.)
    else:
        start = time.monotonic()
        assert not bounded_wait(p, .003)
        assert time.monotonic() - start < .5


@pytest.mark.req('REQ-REC-002', 'REQ-REC-003')
@pytest.mark.tc('TC-REC-002-02')
@pytest.mark.level('U')
def test_logger_serializes_callbacks_and_surfaces_write_failure():
    env = load('record_logger_node.py', ['_serialized'])
    inside, release, second = threading.Event(), threading.Event(), threading.Event()
    n = NS(_io_lock=threading.RLock(), get_logger=lambda: NS(warning=lambda x: None))

    @env['_serialized']
    def callback(self, first):
        if first:
            inside.set()
            assert release.wait(2.)
        else:
            second.set()
            raise OSError('disk full')
    t1 = threading.Thread(target=callback, args=(n, True))
    t2 = threading.Thread(target=callback, args=(n, False))
    t1.start()
    assert inside.wait(1.)
    t2.start()
    assert not second.wait(.02)
    release.set()
    t1.join(1.)
    t2.join(1.)
    assert second.is_set() and n.storage_error == 'disk full'


@pytest.mark.req('REQ-ABT-004')
@pytest.mark.tc('TC-ABT-004-02')
@pytest.mark.level('U')
def test_late_acceptance_callback_cancels_without_another_poll():
    from concurrent.futures import Future
    p = port()
    acceptance, result = Future(), Future()
    sent, cancelled, callbacks = [], [], []
    gh = NS(accepted=True, get_result_async=lambda: result,
            cancel_goal_async=lambda: cancelled.append(1))
    def send(goal, feedback_callback):
        callbacks.append(feedback_callback)
        sent.append(goal)
        return acceptance
    p.n.actions = {'pick': NS(server_is_ready=lambda: True, send_goal_async=send)}
    p._wait = lambda pred, deadline: pred()
    assert p._send('pick', NS(), 1.) is None
    assert p._send('pick', NS(), 1.) is None
    assert len(sent) == 1
    acceptance.set_result(gh)
    assert cancelled == [1]
    assert not p.cancel_active(.001)
    assert p._send('pick', NS(), 1.) is None
    result.set_result(NS(status=5))
    assert p.cancel_active(.01)
    p.reset(NS(is_cancel_requested=False))
    callbacks[0](NS(feedback='stale'))
    assert p._feedback == []


@pytest.mark.req('REQ-ABT-004')
@pytest.mark.tc('TC-ABT-004-02')
@pytest.mark.level('U')
def test_failed_terminal_future_is_not_confirmation():
    from concurrent.futures import Future
    p = port()
    result = Future()
    result.set_exception(RuntimeError('transport lost'))
    p._goal = NS(cancel_goal_async=lambda: None)
    p._result_future = result
    assert not p.cancel_active(.01)
    assert p._goal is not None


@pytest.mark.req('REQ-ABT-009')
@pytest.mark.tc('TC-ABT-009-03')
@pytest.mark.level('U')
@pytest.mark.parametrize('failure', ['rejected', 'exception'])
def test_supervisor_retries_failed_response(failure):
    from concurrent.futures import Future
    stop = NS(Request=lambda **kw: NS(**kw))
    stop.Request.SUPERVISOR = 1
    cls = load('safety_supervisor_node.py', ['SafetySupervisorNode'], Stop=stop)['SafetySupervisorNode']
    n = cls.__new__(cls)
    n.monitor = NS(check=lambda: None, active_fault=lambda: 'ROBOT_COMM_LOST')
    now = [0.]
    n.steady_clock = NS(steady_now=lambda: now[0])
    n._stop_future, n._stop_reason, n._stop_confirmed, n._next_stop_try = None, None, False, 0.
    n.stop_calls = []
    warnings, calls = [], []
    n.get_logger = lambda: NS(warning=warnings.append)
    def send(req):
        f = Future()
        calls.append(f)
        return f
    n._stop_cli = NS(service_is_ready=lambda: True, call_async=send)
    n._tick()
    if failure == 'rejected':
        calls[0].set_result(NS(stopped=False))
    else:
        calls[0].set_exception(OSError('lost'))
    n._tick()
    assert len(calls) == 1 and warnings
    now[0] = .1
    n._tick()
    assert len(calls) == 2
    calls[1].set_result(NS(stopped=True))
    n._tick()
    now[0] = 1.
    n._tick()
    assert len(calls) == 2 and n._stop_confirmed


@pytest.mark.req('REQ-REC-003')
@pytest.mark.tc('TC-REC-002-02')
@pytest.mark.level('U')
def test_actual_step_callbacks_preserve_writer_order(tmp_path):
    env = load('record_logger_node.py', ['RecordLoggerNode', '_serialized', 'steps_path'],
               step_record_to_dict=lambda msg: {'step_seq': msg.step_seq})
    n = env['RecordLoggerNode'].__new__(env['RecordLoggerNode'])
    n._io_lock = threading.RLock()
    n._root, n._date, n.written = str(tmp_path), '2026-11-09', 0
    n.get_logger = lambda: NS(warning=lambda x: None)
    entered, release, second_attempt = threading.Event(), threading.Event(), threading.Event()
    def makedirs(*args, **kw):
        if threading.current_thread().name == 'first':
            entered.set()
            assert release.wait(2.)
        else:
            second_attempt.set()
        os.makedirs(*args, **kw)
    env['os'] = NS(path=os.path, makedirs=makedirs)
    t1 = threading.Thread(target=n._on_step_record, args=(NS(cycle_id='c', step_seq=1),), name='first')
    t2 = threading.Thread(target=n._on_step_record, args=(NS(cycle_id='c', step_seq=2),), name='second')
    t1.start()
    assert entered.wait(1.)
    t2.start()
    blocked = not second_attempt.wait(.05)
    release.set()
    t1.join(1.)
    t2.join(1.)
    records = (tmp_path / n._date / 'c.steps.jsonl').read_text().splitlines()
    assert blocked and [json.loads(line)['step_seq'] for line in records] == [1, 2]
    assert n.written == 2


@pytest.mark.req('REQ-REC-002')
@pytest.mark.tc('TC-REC-002-02')
@pytest.mark.level('U')
@pytest.mark.parametrize('failure', ['open', 'flush'])
def test_trial_open_and_flush_failures_do_not_commit_dedup(tmp_path, failure):
    env = load('record_logger_node.py', ['RecordLoggerNode', '_serialized', 'build_trial_record',
               '_clean_scenario_name', 'trials_path'], validate_trial_record=lambda rec: [])
    n = env['RecordLoggerNode'].__new__(env['RecordLoggerNode'])
    n._io_lock = threading.RLock()
    n._root, n._date = str(tmp_path), '2026-11-09'
    n._env, n._gate, n._git_sha, n._scenario_name, n._seed = 'mock', 'M0', 'sha', 'nominal', 0
    n.assumed_params, n._last_trial_id, n.trials_written = ['x'], '', 0
    n.get_logger = lambda: NS(warning=lambda msg: None)
    class BrokenFile:
        def __enter__(self):
            return self
        def __exit__(self, *args):
            return False
        def write(self, line):
            pass
        def flush(self):
            raise OSError('flush failed')
    def failing_open(*args, **kwargs):
        if failure == 'open':
            raise OSError('open failed')
        return BrokenFile()
    env['open'] = failing_open
    n._write_trial_from_json('{"trial_id":"retry"}')
    assert n.storage_error and n._last_trial_id == '' and n.trials_written == 0
    del env['open']
    n._write_trial_from_json('{"trial_id":"retry"}')
    assert not n.storage_error and n._last_trial_id == 'retry' and n.trials_written == 1


@pytest.mark.req('REQ-ABT-008', 'REQ-ORC-010')
@pytest.mark.tc('TC-ABT-003-03')
@pytest.mark.level('U')
def test_wait_backward_ros_clock_uses_scaled_steady_budget():
    p = port()
    tick = [0.0]
    p.n.cfg_dict = {'watchdog': {'cycle_steady_factor_ratio': 2.0}}
    p.clock.ros_now = lambda: tick[0] - (0.8 if tick[0] >= 0.2 else 0.0)
    p._wait.__globals__['time'] = NS(
        monotonic=lambda: tick[0], sleep=lambda dt: tick.__setitem__(0, tick[0] + .01))
    # A 0.36 s ROS phase plus a 0.8 s backward correction remains before
    # both its ROS deadline (1 s) and its independent steady cap (2 s).
    assert p._wait(lambda: p.clock.ros_now() >= .36, 1.0)
    assert 1.15 <= tick[0] <= 1.18
    tick[0] = 0.0
    p.clock.ros_now = lambda: 0.0
    assert not p._wait(lambda: False, 1.0)
    assert 2.0 <= tick[0] <= 2.02


@pytest.mark.req('REQ-REC-003')
@pytest.mark.tc('TC-REC-003-01')
@pytest.mark.level('U')
def test_step_record_preserves_backward_ros_stamps():
    env = load('record_logger_node.py', ['_time_to_s', 'step_record_to_dict'], StepRecord=object)
    msg = NS(header=NS(stamp=NS(sec=7, nanosec=500000000)), cycle_id='c', step_seq=1,
             step_name='DETECT_STONE', t_start=NS(sec=8, nanosec=0),
             t_end=NS(sec=7, nanosec=500000000), result=0, reason_code='', attempt=1,
             abort_level=0, metric_keys=['stone_count'], metric_values=[1.0])
    row = env['step_record_to_dict'](msg)
    assert (row['t_start_s'], row['t_end_s']) == (8.0, 7.5)
    assert row['metrics'] == {'stone_count': 1.0}


@pytest.mark.req('REQ-ORC-010')
@pytest.mark.tc('TC-ORC-010-02')
@pytest.mark.level('U')
def test_ready_cleanup_result_after_original_deadline():
    p = port()
    p.clock.ros_now = lambda: 91.
    p._released = True
    p.raise_fault(Event.CYCLE_TIMEOUT)
    assert p._wait(lambda: True, 90.)
    p._phase_done.__func__.__globals__['metrics_of'] = lambda fb: dict(zip(fb.metric_keys, fb.metric_values))
    p._feedback = [(90., NS(phase='RETREAT', phase_done=True, metric_keys=[], metric_values=[]))]
    p._result_future = NS(done=lambda: True, result=lambda: NS(result=NS(result=0)))
    assert p._phase_done('RETREAT', 90.) == {}
    assert p._result(90.).result == 0


@pytest.mark.req('REQ-REC-002')
@pytest.mark.tc('TC-REC-002-03')
@pytest.mark.level('U')
def test_schema_found_from_installed_share(tmp_path, monkeypatch):
    import sys
    share = tmp_path / 'share'
    (share / 'schema').mkdir(parents=True)
    schema = share / 'schema/trial_schema.json'
    schema.write_text('{}')
    monkeypatch.chdir(tmp_path)
    monkeypatch.setitem(sys.modules, 'ament_index_python', NS())
    monkeypatch.setitem(sys.modules, 'ament_index_python.packages',
                        NS(get_package_share_directory=lambda name: str(share)))
    env = load('record_logger_node.py', ['_find_schema_path'])
    env['__file__'] = str(tmp_path / 'lib/python/site-packages/orchestrator/record_logger_node.py')
    assert env['_find_schema_path']() == str(schema)


@pytest.mark.req('REQ-ORC-010')
@pytest.mark.tc('TC-ORC-010-02')
@pytest.mark.level('U')
@pytest.mark.parametrize('delayed', [False, True])
def test_runner_cleanup_with_production_ros_port(fake_clock, orch_cfg, delayed):
    from dataclasses import asdict
    from fakes import measures as M
    from fakes.skills import FakeSkillPort
    from orchestrator import step_judges as sj
    from orchestrator.common import State
    from orchestrator.cycle_runner import CycleRunner, DropGoal, OrchestratorCfg
    helpers = load('orchestrator_node.py', ['metrics_of', 'meas_from_metrics'])
    p = port()
    p.clock = fake_clock
    p._released = True
    env = p.place_feedback.__func__.__globals__
    env.update(sj=sj, metrics_of=helpers['metrics_of'],
               meas_from_metrics=helpers['meas_from_metrics'])
    end = fake_clock.ros_now() + 90.
    ready_checks = []
    class CoupledPort(FakeSkillPort):
        def place_feedback(self, phase, deadline):
            if phase != 'RETREAT':
                return super().place_feedback(phase, deadline)
            # Production runner passes its fresh cleanup deadline to the real port.
            def terminal_ready():
                ready_checks.append(1)
                if delayed and len(ready_checks) < 3:
                    fake_clock.advance_ros(.01)
                    return False
                m = asdict(M.OK_RET)
                p._feedback = [(fake_clock.ros_now(), NS(phase='RETREAT', phase_done=True,
                                metric_keys=list(m), metric_values=list(m.values())))]
                return True
            p._feedback = []
            p._result_future = NS(done=terminal_ready, result=lambda: NS(result=NS(result=0)))
            return p.place_feedback(phase, deadline)
    coupled = CoupledPort(fake_clock)
    def on_state(state):
        if state is State.RETREAT:
            fake_clock.advance_ros(end - fake_clock.ros_now() + .1)
            p.raise_fault(Event.CYCLE_TIMEOUT)
    out = CycleRunner(coupled, fake_clock, OrchestratorCfg.from_dict(orch_cfg),
                      on_state=on_state).run(DropGoal(mode=Mode.SYNCHRONIZED,
                          max_belt_speed_mps=.06, timeout_s=90., max_passes=2))
    assert out.result == 'SUCCESS'
    assert out.states[-3:] == [State.RETREAT, State.VERIFY, State.DONE]
    assert coupled.names().count('VerifyDrop') == 1
    assert 'Stop' not in coupled.names()


@pytest.mark.req('REQ-ORC-010')
@pytest.mark.tc('TC-ORC-010-02')
@pytest.mark.level('U')
def test_released_cleanup_frozen_clock_is_still_bounded():
    p = port()
    p._released = True
    p.raise_fault(Event.CYCLE_TIMEOUT)
    start = time.monotonic()
    assert not p._wait(lambda: False, .003)
    assert .006 <= time.monotonic() - start < .5
    p.raise_fault(Event.PROTECTIVE_STOP)
    assert not p._wait(lambda: True, 0.)


# ---------------------------------------------------------------- TC-ORC-011-01
def _finished(p, reason, result=4):
    p._goal = object()
    p._result_future = NS(done=lambda: True, result=lambda: NS(result=NS(result=result, reason_code=reason)))
    p._wait = lambda pred, deadline: pred()


@pytest.mark.req('REQ-ORC-011')
@pytest.mark.tc('TC-ORC-011-01')
@pytest.mark.level('U')
@pytest.mark.parametrize('reason,expected', [('STOPPED', Event.ROBOT_STOPPED),
                                             ('PROTECTIVE_STOP', Event.PROTECTIVE_STOP),
                                             ('CANCELED', None), ('', None)])
def test_stopped_skill_result_latches_hard_fault(reason, expected):
    p = port()
    _finished(p, reason)
    assert p._result(1.).reason_code == reason
    assert p.pending_fault() is expected
    assert expected is None or expected in HARD_FAULTS


@pytest.mark.req('REQ-ORC-011')
@pytest.mark.tc('TC-ORC-011-01')
@pytest.mark.level('U')
def test_stopped_result_reported_by_phase_wait():
    p = port()
    _finished(p, 'STOPPED')
    p._feedback = []
    assert p._phase_done('APPROACH', 1.) is None
    assert p.pending_fault() is Event.ROBOT_STOPPED


@pytest.mark.req('REQ-ORC-011')
@pytest.mark.tc('TC-ORC-011-01')
@pytest.mark.level('U')
def test_specific_cause_wins_over_generic_stop():
    # 자체 감시가 이미 원인을 보고 있으면 그 원인이 남는다
    p = port()
    p.n._watch = lambda: p.raise_fault(Event.CONTROLLER_HEARTBEAT_LOST)
    _finished(p, 'STOPPED')
    p._result(1.)
    assert p.pending_fault() is Event.CONTROLLER_HEARTBEAT_LOST
    # 원인이 나중에 확인되면 일반 정지를 대체한다
    q = port()
    _finished(q, 'STOPPED')
    q._result(1.)
    q.raise_fault(Event.PROTECTIVE_STOP)
    assert q.pending_fault() is Event.PROTECTIVE_STOP
