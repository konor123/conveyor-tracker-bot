"""Two-sided CUSUM of the normalized longitudinal position innovation.

기본값(k 2, h 16, recover 0.5 s)은 실제 벨트의 속도 맥동에는 거의 울리지 않고 20 mm/s 이상 계단은
1 s 안에 잡도록 OFF-1 벨트 영상으로 정했다.
"""


class SpeedChangeDetector:
    def __init__(self, cusum_k=2., cusum_h=16., recover_s=.5):
        self.cusum_k = cusum_k
        self.cusum_h = cusum_h
        self.recover_s = recover_s
        self.g_plus = self.g_minus = 0.
        self.last_change_stamp = None
        self.steady = True

    def advance(self, t):
        if self.last_change_stamp is not None and t-self.last_change_stamp >= self.recover_s:
            self.steady = True

    def update(self, t, innovation_v):
        self.advance(t)
        self.g_plus = max(0., self.g_plus + innovation_v - self.cusum_k)
        self.g_minus = max(0., self.g_minus - innovation_v - self.cusum_k)
        if max(self.g_plus, self.g_minus) > self.cusum_h:
            self.g_plus = self.g_minus = 0.
            self.last_change_stamp = t
            self.steady = False
            return True
        return False
