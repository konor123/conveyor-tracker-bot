"""record_logger_node: 단계·시도 기록 노드.

- StepRecord(task/step_record) → records/trials/<date>/<cycle_id>.steps.jsonl
- 사이클 완료(orchestrator_node last_trial_json 파라미터 이벤트) → records/trials/<date>/trials.jsonl
- 대역 노드의 assumed_params·scenario·seed 파라미터를 읽어 시도 기록에 복사한다.
"""
from __future__ import annotations

import datetime
import json
import os
import subprocess
import threading
from functools import wraps

import rclpy
from drop_interfaces.msg import StepRecord
from rcl_interfaces.msg import ParameterEvent, ParameterType
from rcl_interfaces.srv import GetParameters
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy

from .ros_qos import no_sub_events
from .node_shutdown import ShutdownGuard


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


def trials_path(root: str, date: str) -> str:
    if '/' in date or '..' in date:
        raise ValueError(f'잘못된 date: {date!r}')
    return os.path.join(root, date, 'trials.jsonl')


def _clean_scenario_name(value: str) -> str:
    base = os.path.basename(str(value or 'nominal_20mms'))
    return base[:-5] if base.endswith('.yaml') else base


def build_trial_record(
    summary: dict,
    *,
    env: str = 'mock',
    gate: str = 'M0',
    git_sha: str = 'unknown',
    scenario_name: str = 'nominal_20mms',
    seed: int = 0,
    assumed_params: list[str] | None = None,
    interface_version: str = 'drop_interfaces 0.2',
) -> dict:
    params = [p for p in (assumed_params or []) if p]
    return {
        'trial_id': str(summary.get('trial_id', '')),
        'env': str(env),
        'interface_version': str(interface_version),
        'scenario': {
            'name': _clean_scenario_name(summary.get('scenario_name') or scenario_name),
            'seed': int(summary.get('seed', seed)),
        },
        'assumed_params': params,
        'gate': str(gate),
        'git_sha': str(git_sha),
        'mode': str(summary.get('mode', 'MODE_SYNCHRONIZED')),
        'result': str(summary.get('result', 'NOT_ATTEMPTED')),
        'abort_level': max(0, min(4, int(summary.get('abort_level', 0)))),
        'reason_code': str(summary.get('reason_code', '')),
        'passes_skipped': max(0, int(summary.get('passes_skipped', 0))),
        'cycle_time_s': float(summary.get('cycle_time_s', 0.0)),
        'step_durations_s': dict(summary.get('step_durations_s') or {}),
    }


def _find_schema_path() -> str | None:
    candidates = [
        os.path.join(os.path.dirname(__file__), *[os.pardir] * 3, 'tools', 'eval', 'trial_schema.json'),
        os.path.join(os.getcwd(), 'tools', 'eval', 'trial_schema.json'),
    ]
    try:
        from ament_index_python.packages import get_package_share_directory
        candidates.append(os.path.join(get_package_share_directory('orchestrator'),
                                       'schema', 'trial_schema.json'))
    except (ImportError, LookupError):
        pass
    for p in candidates:
        if os.path.isfile(p):
            return os.path.realpath(p)
    return None


def validate_trial_record(record: dict) -> list[str]:
    schema_path = _find_schema_path()
    if schema_path is None:
        return ['trial schema unavailable']
    from jsonschema import Draft202012Validator
    with open(schema_path, encoding='utf-8') as fh:
        schema = json.load(fh)
    validator = Draft202012Validator(schema)
    return [e.message for e in validator.iter_errors(record)]


def _default_git_sha() -> str:
    try:
        out = subprocess.check_output(['git', 'rev-parse', '--short', 'HEAD'],
                                      stderr=subprocess.DEVNULL, text=True, timeout=2.0).strip()
        return out or 'unknown'
    except Exception:
        return 'unknown'


def _serialized(method):
    @wraps(method)
    def guarded(self, *args, **kwargs):
        with self._io_lock:
            try:
                return method(self, *args, **kwargs)
            except OSError as exc:
                self.storage_error = str(exc)
                self.get_logger().warning(f'record storage failed: {exc}')
    return guarded


def repo_path(value: str) -> str:
    """상대 경로는 저장소 루트 기준(원본 구현 calibration.repo_paths와 같은 규칙).

    루트: DROP_REPO_ROOT → 이 파일에서 위로 올라가며 src·config를 함께 가진 첫 폴더 → 현재 폴더.
    """
    if not value:
        return value
    value = os.path.expanduser(value)
    if os.path.isabs(value):
        return value
    root = os.environ.get('DROP_REPO_ROOT')
    if not root:
        d = os.path.dirname(os.path.abspath(__file__))
        while True:
            if os.path.isdir(os.path.join(d, 'src')) and os.path.isdir(os.path.join(d, 'config')):
                root = d
                break
            parent = os.path.dirname(d)
            if parent == d:
                root = os.getcwd()
                break
            d = parent
    return os.path.normpath(os.path.join(os.path.expanduser(root), value))


class RecordLoggerNode(Node):
    def __init__(self, **kwargs):
        super().__init__('record_logger_node', **kwargs)
        self._io_lock = threading.RLock()
        self.storage_error = ''
        default_dir = os.environ.get('DROP_RECORDS_DIR') or os.path.join('records', 'trials')
        self.declare_parameter('records_dir', default_dir)
        self.declare_parameter('date', '')
        self.declare_parameter('env', 'mock')
        self.declare_parameter('gate', 'M0')
        self.declare_parameter('git_sha', '')
        self.declare_parameter('scenario', 'nominal_20mms')
        self.declare_parameter('seed', 0)
        self.declare_parameter('mock_node', 'mock_world_node')
        self._root = repo_path(str(self.get_parameter('records_dir').value))
        date = str(self.get_parameter('date').value)
        self._date = date or datetime.date.today().isoformat()
        self._env = str(self.get_parameter('env').value)
        self._gate = str(self.get_parameter('gate').value)
        self._git_sha = str(self.get_parameter('git_sha').value) or _default_git_sha()
        self._scenario_name = _clean_scenario_name(str(self.get_parameter('scenario').value))
        self._seed = max(0, int(self.get_parameter('seed').value))
        self.assumed_params: list[str] = []
        self._mock_synced = False
        self._poll_inflight = False
        self._last_trial_id = ''
        self.written = 0
        self.trials_written = 0
        group = ReentrantCallbackGroup()
        qos = QoSProfile(depth=100, reliability=ReliabilityPolicy.RELIABLE,
                         durability=DurabilityPolicy.VOLATILE)
        mock_node = str(self.get_parameter('mock_node').value).lstrip('/')
        self._mock_cli = self.create_client(GetParameters, f'/{mock_node}/get_parameters',
                                            callback_group=group)
        self.create_subscription(StepRecord, 'task/step_record', self._on_step_record, qos,
                                 callback_group=group, event_callbacks=no_sub_events())
        self.create_subscription(ParameterEvent, '/parameter_events', self._on_param_event, qos,
                                 callback_group=group, event_callbacks=no_sub_events())
        self.create_timer(0.1, self._poll_mock_params, callback_group=group)

    @_serialized
    def _poll_mock_params(self) -> None:
        if self._mock_synced or self._poll_inflight or not self._mock_cli.service_is_ready():
            return
        self._poll_inflight = True
        req = GetParameters.Request(names=['scenario', 'seed', 'assumed_params'])
        fut = self._mock_cli.call_async(req)
        fut.add_done_callback(self._on_mock_params_reply)

    @_serialized
    def _on_mock_params_reply(self, fut) -> None:
        self._poll_inflight = False
        try:
            res = fut.result()
        except Exception:
            return
        if res is None or len(res.values) < 3:
            return
        v_scen, v_seed, v_assumed = res.values[0], res.values[1], res.values[2]
        if v_scen.type == ParameterType.PARAMETER_STRING and v_scen.string_value:
            self._scenario_name = _clean_scenario_name(v_scen.string_value)
        if v_seed.type == ParameterType.PARAMETER_INTEGER and v_seed.integer_value >= 0:
            self._seed = int(v_seed.integer_value)
        if v_assumed.type == ParameterType.PARAMETER_STRING_ARRAY:
            self.assumed_params = [s for s in v_assumed.string_array_value if s]
            self._mock_synced = True

    @_serialized
    def _on_param_event(self, ev: ParameterEvent) -> None:
        node = ev.node.lstrip('/')
        all_params = list(ev.new_parameters) + list(ev.changed_parameters)
        if node in ('mock_world_node', 'mock_robot_controller_node'):
            for p in all_params:
                v = p.value
                if p.name == 'scenario' and v.type == ParameterType.PARAMETER_STRING and v.string_value:
                    self._scenario_name = _clean_scenario_name(v.string_value)
                elif p.name == 'seed' and v.type == ParameterType.PARAMETER_INTEGER and v.integer_value >= 0:
                    self._seed = int(v.integer_value)
                elif p.name == 'assumed_params' and v.type == ParameterType.PARAMETER_STRING_ARRAY:
                    self.assumed_params = [s for s in v.string_array_value if s]
                    self._mock_synced = True
        elif node == 'orchestrator_node':
            for p in all_params:
                if p.name == 'last_trial_json' and p.value.type == ParameterType.PARAMETER_STRING:
                    raw = p.value.string_value
                    if raw:
                        self._write_trial_from_json(raw)

    def _ensure_assumed_params(self) -> list[str]:
        if self.assumed_params:
            return self.assumed_params
        if self._env == 'mock':
            try:
                from drop_mocks.ros_common import scenario_file
                from drop_mocks.scenario_engine import ScenarioEngine
                self.assumed_params = ScenarioEngine.from_file(
                    scenario_file(self._scenario_name), self._seed).assumed_params()
            except Exception:
                pass
        return self.assumed_params

    @_serialized
    def _write_trial_from_json(self, raw: str) -> None:
        try:
            summary = json.loads(raw)
            if not isinstance(summary, dict):
                raise ValueError('summary must be an object')
        except Exception as e:
            self._quarantine({'raw': raw, 'notes': str(e)})
            self.get_logger().warning(f'last_trial_json 파싱 실패: {e}')
            return
        tid = str(summary.get('trial_id', ''))
        if tid and tid == self._last_trial_id:
            return
        try:
            rec = build_trial_record(
                summary,
                env=self._env,
                gate=self._gate,
                git_sha=self._git_sha,
                scenario_name=self._scenario_name,
                seed=self._seed,
                assumed_params=self._ensure_assumed_params(),
            )
        except (TypeError, ValueError, OverflowError) as exc:
            self._quarantine({'raw': raw, 'notes': str(exc)})
            return
        errors = validate_trial_record(rec)
        try:
            path = trials_path(self._root, self._date)
        except ValueError as e:
            self.get_logger().warning(f'trials_path 오류: {e}')
            return
        os.makedirs(os.path.dirname(path), exist_ok=True)
        if errors:
            rec['notes'] = '; '.join(errors)
            bad_path = path[:-6] + '.invalid.jsonl'
            with open(bad_path, 'a', encoding='utf-8') as fh:
                fh.write(json.dumps(rec, ensure_ascii=False) + '\n')
                fh.flush()
            self.get_logger().warning(f'시도 기록 스키마 오류 보관: {errors}')
            return
        with open(path, 'a', encoding='utf-8') as fh:
            fh.write(json.dumps(rec, ensure_ascii=False) + '\n')
            fh.flush()
        self._last_trial_id = tid
        self.storage_error = ''
        self.trials_written += 1

    def _quarantine(self, record):
        path = trials_path(self._root, self._date)[:-6] + '.invalid.jsonl'
        os.makedirs(os.path.dirname(path), exist_ok=True)
        with open(path, 'a', encoding='utf-8') as fh:
            fh.write(json.dumps(record, ensure_ascii=False) + '\n')
            fh.flush()

    @_serialized
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
    guard = ShutdownGuard()            # rclpy.init 전
    rclpy.init(args=args)
    node = None
    try:
        node = RecordLoggerNode()
        executor = MultiThreadedExecutor(num_threads=4)
        executor.add_node(node)
        executor.spin()
    except BaseException as exc:
        if not guard.is_shutdown(exc):
            raise
    finally:
        if node is not None:
            node.destroy_node()
        rclpy.try_shutdown()
        guard.close()


if __name__ == '__main__':
    main()
