"""tools/bench/gripper_pulse_table.py 단위 테스트(ROS 불필요).

    python3 -m unittest tools/bench/test_gripper_pulse_table.py -v
"""
import csv
import os
import sys
import tempfile
import unittest

sys.path.insert(0, os.path.dirname(__file__))
import gripper_pulse_table as g  # noqa: E402


def linear_gap(p):
    """FakeGripper 기본 선형 표(TEST_SPEC 3.9절): 0 pulse = 106 mm, 1150 pulse = 0 mm."""
    return 106.0 * (1 - p / 1150)


class Codec(unittest.TestCase):
    def test_roundtrip_all(self):
        for v in range(0, 0x4000):
            hi, lo = g.encode7(v)
            self.assertTrue(0 <= hi <= 127 and 0 <= lo <= 127)
            self.assertEqual(g.decode7(hi, lo), v)

    def test_out_of_range(self):
        for v in (-1, 0x4000, 1.5):
            with self.assertRaises(ValueError):
                g.encode7(v)

    def test_seq_changes_every_time(self):
        s = 63
        for _ in range(500):
            n = g.next_seq(s)
            self.assertNotEqual(n, s)
            self.assertTrue(1 <= n <= 127)
            s = n


class Writer(unittest.TestCase):
    def test_payload_then_trigger(self):
        t = g.DryTransport()
        w = g.RegisterWriter(t.write)
        seq = w.send(g.Command(1100, 100))
        self.assertEqual([a for a, _ in t.log], [11, 12, 13, 14, 10])
        self.assertEqual(g.decode7(t.log[0][1], t.log[1][1]), 1100)   # 750 제한·반전 없음
        self.assertEqual(g.decode7(t.log[2][1], t.log[3][1]), 100)
        self.assertEqual(t.log[4][1], seq)

    def test_rejects_out_of_range(self):
        w = g.RegisterWriter(g.DryTransport().write)
        for c in (g.Command(1151, 100), g.Command(-1, 100), g.Command(100, 0), g.Command(100, 101)):
            with self.assertRaises(ValueError):
                w.send(c)

    def test_failed_write_raises_before_trigger(self):
        log = []

        def write(a, v):
            log.append(a)
            return a != 12
        with self.assertRaises(RuntimeError):
            g.RegisterWriter(write).send(g.Command(500, 100))
        self.assertNotIn(10, log)


class Points(unittest.TestCase):
    def test_coarse(self):
        pts = g.coarse_points()
        self.assertEqual(pts[0], 0)
        self.assertEqual(pts[-1], 1150)
        self.assertEqual(len(pts), 13)

    def test_fine_range_linear(self):
        samples = [(p, linear_gap(p)) for p in g.coarse_points()]
        fine = g.fine_range(samples, 15.0)
        # 5 mm ↔ 1095.8 pulse, 25 mm ↔ 878.8 pulse → 875~1100
        self.assertEqual(fine[0], 875)
        self.assertEqual(fine[-1], 1100)
        self.assertTrue(all(p % 25 == 0 for p in fine))

    def test_fine_range_one_end_outside(self):
        samples = [(p, linear_gap(p)) for p in range(0, 901, 100)]   # 900 pulse(23 mm)까지만 측정
        fine = g.fine_range(samples, 15.0)
        self.assertEqual(fine[-1], 900)

    def test_fine_range_needs_two_samples(self):
        self.assertEqual(g.fine_range([(0, 106.0)], 15.0), [])


class Session(unittest.TestCase):
    def run_main(self, answers, extra=()):
        tmp = tempfile.mkdtemp()
        out = os.path.join(tmp, "t.csv")
        it = iter(answers)
        t = g.DryTransport()
        rc = g.main(["--stone-mm", "15", "--out", out, "--passes-up", "1", "--passes-down", "1", *extra],
                    inp=lambda _: next(it), transport=t, sleep=lambda _: None)
        with open(out) as fh:
            rows = list(csv.DictReader(fh))
        return rc, rows, t

    def test_full_session(self):
        coarse = g.coarse_points()
        survey = [f"{linear_gap(p):.2f}" for p in coarse]
        pts = g.merge_points(coarse, g.fine_range([(p, linear_gap(p)) for p in coarse], 15.0))
        up = [f"{linear_gap(p):.2f}" for p in pts]
        down = [f"{linear_gap(p):.2f}" for p in reversed(pts)]
        rc, rows, t = self.run_main(["yes", *survey, *up, *down])
        self.assertEqual(rc, 0)
        self.assertEqual(len(rows), len(coarse) + 2 * len(pts))
        downs = [int(r["cmd_pulse"]) for r in rows if r["direction"] == "down"]
        self.assertEqual(downs, sorted(downs, reverse=True))
        last_pos = g.decode7(t.log[-5][1], t.log[-4][1])
        self.assertEqual(last_pos, 0)

    def test_quit_saves_and_returns_open(self):
        rc, rows, t = self.run_main(["yes", "106", "96.8", "q"])
        self.assertEqual(rc, 0)
        self.assertEqual(len(rows), 2)
        self.assertEqual(g.decode7(t.log[-5][1], t.log[-4][1]), 0)

    def test_resend_and_skip(self):
        rc, rows, _ = self.run_main(["yes", "r", "106", "", "q"])
        self.assertEqual([r["gap_mm"] for r in rows], ["106.0", ""])

    def test_requires_yes(self):
        it = iter(["no"])
        rc = g.main(["--stone-mm", "15", "--out", os.path.join(tempfile.mkdtemp(), "x.csv")],
                    inp=lambda _: next(it), transport=g.DryTransport(), sleep=lambda _: None)
        self.assertEqual(rc, 1)


if __name__ == "__main__":
    unittest.main()
