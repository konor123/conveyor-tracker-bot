"""tools/bench/bag_integrity.py 단위 시험(ROS 없이 합성 메시지)."""
import os
import struct
import sys

import pytest

sys.path.insert(0, os.path.dirname(__file__))
import bag_integrity as bi  # noqa: E402

IMG, INFO = "/c/image_raw", "/c/camera_info"
FRAME = 33_333_333  # 30 Hz [ns]


def _stream(topic, n, start_stamp=10**12, lag_ns=20_000_000, step=FRAME, bag0=None):
    bag0 = start_stamp + lag_ns if bag0 is None else bag0
    return [bi.BagMessage(topic, bag0 + i * step, start_stamp + i * step) for i in range(n)]


def _analyze(msgs):
    return bi.analyze_bag_messages(msgs, [(IMG, INFO)])


@pytest.mark.level("U")
def test_clean_stream_has_no_gaps_or_backwards():
    r = _analyze(_stream(IMG, 300) + _stream(INFO, 300))
    s = r["streams"][IMG]
    assert s["count"] == 300 and s["stamp_backward"] == 0 and s["stamp_repeat"] == 0
    assert s["gaps_over_50ms"] == 0 and s["lag_over_0_3s"] == 0
    assert s["lag_p50_s"] == pytest.approx(0.020) and not s["first_retained_old"]
    assert s["avg_fps"] == pytest.approx(30.0, rel=0.01)
    assert r["pairs"][f"{IMG}|{INFO}"]["paired"] == 300


@pytest.mark.level("U")
def test_backward_stamp_is_counted_in_arrival_order():
    msgs = _stream(IMG, 10)
    msgs[5].stamp_ns = msgs[3].stamp_ns  # 도착 순서에서 stamp가 뒤로 감
    s = _analyze(msgs)["streams"][IMG]
    assert s["stamp_backward"] == 1  # 4→5 에서만 내려가고 5→6 은 다시 오른다


@pytest.mark.level("U")
def test_repeated_stamp_counted_separately():
    msgs = _stream(IMG, 6)
    msgs[3].stamp_ns = msgs[2].stamp_ns
    s = _analyze(msgs)["streams"][IMG]
    assert s["stamp_repeat"] == 1


@pytest.mark.level("U")
def test_gap_over_50ms_and_max_gap():
    msgs = _stream(IMG, 10)
    for m in msgs[5:]:
        m.stamp_ns += 100_000_000  # 4→5 간격 약 133 ms
        m.bag_time_ns += 100_000_000
    s = _analyze(msgs)["streams"][IMG]
    assert s["gaps_over_50ms"] == 1
    assert s["max_gap_s"] == pytest.approx(0.1333, abs=0.001)


@pytest.mark.level("U")
def test_leading_retained_sample_is_flagged_and_excluded_gap_reported():
    msgs = _stream(IMG, 10, start_stamp=10**12 + 50 * 10**9)
    old = bi.BagMessage(IMG, msgs[0].bag_time_ns, 10**12)  # 50 s 오래된 선두 샘플
    s = _analyze([old] + msgs)["streams"][IMG]
    assert s["first_retained_old"] and s["first_lag_s"] > 49
    assert s["gaps_over_50ms"] == 1                       # 선두 1장이 간격을 만든다
    assert s["excluding_first"]["gaps_over_50ms"] == 0     # 선두를 빼면 0
    assert s["lag_over_0_3s"] == 1


@pytest.mark.level("U")
def test_pairing_reports_unpaired_by_position():
    img = _stream(IMG, 10)
    info = _stream(INFO, 10)
    del info[0]          # 영상 선두 1장은 짝 없음
    del info[-1]         # 영상 후미 1장은 짝 없음
    del info[4]          # 영상 중간 1장
    p = _analyze(img + info)["pairs"][f"{IMG}|{INFO}"]
    assert p["paired"] == 7
    where = sorted(u["where"] for u in p["unpaired_images"])
    assert where == ["interior", "leading", "trailing"]
    assert p["unpaired_infos"] == []


@pytest.mark.level("U")
def test_empty_and_single_message():
    assert _analyze([])["streams"] == {}
    s = _analyze(_stream(IMG, 1))["streams"][IMG]
    assert s["count"] == 1 and s["avg_fps"] == 0.0
    p = _analyze(_stream(IMG, 1))["pairs"][f"{IMG}|{INFO}"]
    assert p["paired"] == 0 and p["info_count"] == 0


@pytest.mark.level("U")
@pytest.mark.parametrize("little", [True, False])
def test_parse_header_cdr(little):
    e = "<" if little else ">"
    frame = b"camera_color_optical_frame\x00"
    cdr = bytes([0, 1 if little else 0, 0, 0]) + struct.pack(e + "iII", 1794222000, 123456789, len(frame)) + frame
    stamp, fid = bi.parse_header(cdr + b"\x00" * 8)
    assert stamp == 1794222000 * 10**9 + 123456789
    assert fid == "camera_color_optical_frame"


@pytest.mark.level("U")
def test_parse_header_rejects_short_message():
    with pytest.raises(ValueError):
        bi.parse_header(b"\x00\x01\x00\x00")


@pytest.mark.level("U")
def test_cli_pair_format_is_validated(capsys):
    with pytest.raises(SystemExit):
        bi.main(["/nonexistent", "--out", "/tmp/x.json", "--pair", "only_one_topic"])
