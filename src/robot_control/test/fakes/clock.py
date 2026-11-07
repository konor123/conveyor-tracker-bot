"""테스트용 steady/ROS 시계. U 시험은 실제 시간을 쓰지 않는다."""


class FakeClock:
    def __init__(self, ros: float = 1000.0, steady: float = 50.0):
        self._ros = float(ros)
        self._steady = float(steady)

    def ros_now(self) -> float:
        return self._ros

    def steady_now(self) -> float:
        return self._steady

    def advance(self, dt: float) -> None:
        if dt < 0:
            raise ValueError('advance는 음수를 받지 않는다')
        self._steady += dt
        self._ros += dt

    def advance_ros(self, dt: float) -> None:
        self._ros += dt

    def jump_ros(self, dt: float) -> None:
        self._ros += dt
