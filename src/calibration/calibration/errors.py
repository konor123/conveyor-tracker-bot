"""보정 도구 공통 예외."""


class CalibrationError(ValueError):
    """보정 계산 실패. code: DEGENERATE(퇴화 배치·점 부족), NO_CONSENSUS(RANSAC 합의 부족)."""

    def __init__(self, code: str, message: str):
        super().__init__(f'{code}: {message}')
        self.code = code
