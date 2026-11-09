"""검출 시험용 합성 영상(TEST_SPEC 3.5절 합성 조건). 8배 확대 렌더 후 면적 평균 축소로 경계를 서브픽셀로 만든다."""
import cv2
import numpy as np

W, H = 640, 400
SS = 8                                  # 슈퍼샘플 배율
MAT_BGR = (56, 70, 70)                  # 2026-11-09 OFF-1 매트 중앙값(BGR)
BELT_BGR = (90, 170, 147)               # 벨트 중앙값
CAP_BGR = (201, 152, 37)                # 파란 병뚜껑
STONE_BGR = {'yellow': (140, 255, 255), 'pink': (156, 89, 251), 'purple': (177, 118, 176),
             'red': (72, 46, 194), 'blue': (152, 94, 16)}
STONE_FLAT_PX = 31.0                    # 팔각형 면-면 17 mm / 0.55 mm/px
CAP_OUTER_PX = 58.0                     # 바깥 지름 32 mm
CAP_INNER_RATIO = 0.80


def octagon_area(flat):
    return 2.0 * flat * flat * (np.sqrt(2.0) - 1.0)


def octagon_eq_diameter(flat):
    return float(np.sqrt(4.0 * octagon_area(flat) / np.pi))


class Canvas:
    def __init__(self, bg_bgr, w=W, h=H):
        self.big = np.empty((h * SS, w * SS, 3), np.uint8)
        self.big[:] = bg_bgr
        self.w, self.h = w, h

    def _pt(self, u, v):
        # 축소 뒤 픽셀 i의 중심이 i가 되도록(큰 영상 좌표 = (u + 0.5) * SS - 0.5)
        return (u + 0.5) * SS - 0.5, (v + 0.5) * SS - 0.5

    def octagon(self, u, v, flat, bgr, rot_rad=0.0):
        cu, cv = self._pt(u, v)
        r = 0.5 * flat * SS / np.cos(np.pi / 8)
        a = rot_rad + np.pi / 8 + np.arange(8) * np.pi / 4
        pts = np.column_stack([cu + r * np.cos(a), cv + r * np.sin(a)])
        cv2.fillPoly(self.big, [np.round(pts * 16).astype(np.int32)], bgr, lineType=cv2.LINE_8, shift=4)

    def disk(self, u, v, diameter, bgr):
        cu, cv = self._pt(u, v)
        cv2.circle(self.big, (int(round(cu * 16)), int(round(cv * 16))), int(round(diameter * SS * 8)),
                   bgr, -1, lineType=cv2.LINE_8, shift=4)

    def cap(self, u, v, outer=CAP_OUTER_PX, inner_ratio=CAP_INNER_RATIO, bgr=CAP_BGR):
        self.disk(u, v, outer, bgr)
        # 안쪽 바닥은 약간 밝은 같은 색
        self.disk(u, v, outer * inner_ratio, tuple(min(255, int(c * 1.1) + 10) for c in bgr))

    def square(self, u, v, side, bgr):
        cu, cv = self._pt(u, v)
        h = side * SS / 2
        cv2.rectangle(self.big, (int(cu - h), int(cv - h)), (int(cu + h), int(cv + h)), bgr, -1)

    def image(self, noise_sigma=2.0, rng=None):
        img = cv2.resize(self.big, (self.w, self.h), interpolation=cv2.INTER_AREA).astype(np.float32)
        if noise_sigma > 0:
            rng = rng or np.random.default_rng(0)
            img += rng.normal(0.0, noise_sigma, img.shape[:2])[..., None]
        return np.clip(np.round(img), 0, 255).astype(np.uint8)
