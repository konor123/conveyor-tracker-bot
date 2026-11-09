#!/usr/bin/env python3
"""rosbag2 영상 bag 무결성 검사(수치만 낸다. "무손실" 같은 판정 문구는 쓰지 않는다).

영상 토픽과 CameraInfo 토픽 쌍마다 다음을 계산한다. 기록(도착) 순서대로 센다.
- 개수, 기록 시각 기간, 평균 fps
- header.stamp 역행 횟수(인접 stamp 차 < 0), 같은 stamp 반복 횟수
- 인접 stamp 간격 최대값과 50 ms 초과 횟수. 선두 메시지가 오래된 retained 샘플이면 그 한 장을 뺀 값도 따로 낸다.
- (기록 시각 − stamp) 분포 p50/p95/max와 0.3 s 초과 개수, 선두 메시지의 값
- 영상↔CameraInfo stamp 쌍 수와 짝 없는 메시지(선두/후미/중간 위치 구분)

header는 CDR 앞부분만 읽어 stamp를 얻는다(영상 본문은 역직렬화하지 않는다).

CLI:
  python3 tools/bench/bag_integrity.py <bag_dir> --out <json> [--pair 영상토픽,정보토픽 ...]
bag 읽기는 rosbag2_py(ROS 환경)가 필요하다. 분석 함수는 ROS 없이 시험된다.
"""
from __future__ import annotations

import argparse
import json
import struct
import sys
from dataclasses import dataclass
from typing import Any, Dict, List, Optional, Sequence, Tuple

import numpy as np

GAP_LIMIT_S = 0.050       # 인접 stamp 간격 기준 [s]
STALE_LIMIT_S = 0.3       # (기록 시각 − stamp) 기준 [s]
DEFAULT_PAIRS = (
    ("/camera/camera/color/image_raw", "/camera/camera/color/camera_info"),
    ("/camera/camera/depth/image_rect_raw", "/camera/camera/depth/camera_info"),
)


@dataclass
class BagMessage:
    topic: str
    bag_time_ns: int
    stamp_ns: int
    frame_id: str = ""


def parse_header(cdr: bytes) -> Tuple[int, str]:
    """CDR 직렬화 메시지 앞의 std_msgs/Header에서 (stamp ns, frame_id)를 뽑는다."""
    if len(cdr) < 16:
        raise ValueError("CDR 메시지가 너무 짧다")
    little = cdr[1] == 1
    e = "<" if little else ">"
    sec, nanosec, strlen = struct.unpack_from(e + "iII", cdr, 4)
    frame_id = cdr[16:16 + max(strlen - 1, 0)].decode("utf-8", "replace")
    return sec * 10**9 + nanosec, frame_id


def _stream_stats(msgs: Sequence[BagMessage]) -> Dict[str, Any]:
    n = len(msgs)
    out: Dict[str, Any] = {"count": n}
    if n == 0:
        return out
    bag = np.array([m.bag_time_ns for m in msgs], dtype=np.int64)
    stamp = np.array([m.stamp_ns for m in msgs], dtype=np.int64)
    lag_s = (bag - stamp) / 1e9
    span = (bag.max() - bag.min()) / 1e9
    out.update(
        bag_span_s=float(span),
        avg_fps=float((n - 1) / span) if n > 1 and span > 0 else 0.0,
        stamp_span_s=float((stamp.max() - stamp.min()) / 1e9),
        lag_p50_s=float(np.percentile(lag_s, 50)),
        lag_p95_s=float(np.percentile(lag_s, 95)),
        lag_max_s=float(lag_s.max()),
        lag_over_0_3s=int((lag_s > STALE_LIMIT_S).sum()),
        first_lag_s=float(lag_s[0]),
        first_retained_old=bool(lag_s[0] > STALE_LIMIT_S),
    )

    def gaps(s: np.ndarray) -> Dict[str, Any]:
        d = np.diff(s) / 1e9
        return {
            "stamp_backward": int((d < 0).sum()),
            "stamp_repeat": int((d == 0).sum()),
            "max_gap_s": float(d.max()) if len(d) else 0.0,
            "gaps_over_50ms": int((d > GAP_LIMIT_S).sum()),
        }

    out.update(gaps(stamp))
    if out["first_retained_old"] and n > 1:
        out["excluding_first"] = gaps(stamp[1:])
    return out


def _unpaired(msgs: Sequence[BagMessage], other_stamps: set) -> List[Dict[str, Any]]:
    """다른 토픽에 같은 stamp가 없는 메시지(기록 순서 위치와 선두/후미/중간 구분)."""
    result = []
    n = len(msgs)
    for i, m in enumerate(msgs):
        if m.stamp_ns in other_stamps:
            continue
        where = "leading" if i == 0 else "trailing" if i == n - 1 else "interior"
        result.append({"index": i, "where": where,
                       "bag_minus_stamp_s": (m.bag_time_ns - m.stamp_ns) / 1e9})
    return result


def analyze_bag_messages(messages: Sequence[BagMessage],
                         pairs: Sequence[Tuple[str, str]] = DEFAULT_PAIRS) -> Dict[str, Any]:
    """기록 순서의 메시지 목록에서 토픽별·쌍별 수치를 계산한다(순수 함수)."""
    by_topic: Dict[str, List[BagMessage]] = {}
    for m in messages:
        by_topic.setdefault(m.topic, []).append(m)
    result: Dict[str, Any] = {"streams": {t: _stream_stats(v) for t, v in by_topic.items()}, "pairs": {}}
    for img, info in pairs:
        im, inf = by_topic.get(img, []), by_topic.get(info, [])
        istamps = {m.stamp_ns for m in im}
        fstamps = {m.stamp_ns for m in inf}
        result["pairs"][f"{img}|{info}"] = {
            "paired": len(istamps & fstamps),
            "image_count": len(im),
            "info_count": len(inf),
            "unpaired_images": _unpaired(im, fstamps),
            "unpaired_infos": _unpaired(inf, istamps),
        }
    return result


def read_bag(bag_dir: str, topics: Optional[Sequence[str]] = None) -> List[BagMessage]:
    """rosbag2_py로 (토픽, 기록 시각, stamp, frame_id)를 기록 순서대로 읽는다."""
    import rosbag2_py  # ROS 환경 필요. 분석 함수 시험에는 쓰지 않는다.

    reader = rosbag2_py.SequentialReader()
    reader.open(rosbag2_py.StorageOptions(uri=bag_dir, storage_id=""),
                rosbag2_py.ConverterOptions("cdr", "cdr"))
    wanted = set(topics) if topics else None
    out: List[BagMessage] = []
    while reader.has_next():
        topic, data, bag_ns = reader.read_next()
        if wanted is not None and topic not in wanted:
            continue
        try:
            stamp_ns, frame_id = parse_header(data)
        except (ValueError, struct.error):
            continue  # header가 없는 토픽(/tf_static 등)
        out.append(BagMessage(topic, int(bag_ns), stamp_ns, frame_id))
    return out


def _fmt(v: Any) -> str:
    return f"{v:.3f}" if isinstance(v, float) else str(v)


def print_summary(result: Dict[str, Any]) -> None:
    for topic, s in result["streams"].items():
        print(f"{topic}: " + " ".join(f"{k}={_fmt(v)}" for k, v in s.items() if not isinstance(v, dict)))
        if "excluding_first" in s:
            print("   선두 제외: " + " ".join(f"{k}={_fmt(v)}" for k, v in s["excluding_first"].items()))
    for name, p in result["pairs"].items():
        print(f"{name}: paired={p['paired']} image={p['image_count']} info={p['info_count']} "
              f"unpaired_images={len(p['unpaired_images'])} unpaired_infos={len(p['unpaired_infos'])}")


def main(argv: Optional[Sequence[str]] = None) -> int:
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("bag_dir")
    ap.add_argument("--out", required=True, help="결과 JSON 경로")
    ap.add_argument("--pair", action="append", default=[], metavar="IMG,INFO",
                    help="영상 토픽,CameraInfo 토픽(여러 번 지정 가능). 생략하면 컬러·깊이 기본 쌍")
    args = ap.parse_args(argv)
    pairs = [tuple(p.split(",", 1)) for p in args.pair] or list(DEFAULT_PAIRS)
    if any(len(p) != 2 or not all(p) for p in pairs):
        ap.error("--pair는 '영상토픽,정보토픽' 형식이다")
    topics = sorted({t for p in pairs for t in p})
    result = analyze_bag_messages(read_bag(args.bag_dir, topics), pairs)
    result["bag_dir"] = args.bag_dir
    with open(args.out, "w", encoding="utf-8") as f:
        json.dump(result, f, indent=2, ensure_ascii=False)
    print(f"bag: {args.bag_dir}")
    print_summary(result)
    return 0


if __name__ == "__main__":
    sys.exit(main())
