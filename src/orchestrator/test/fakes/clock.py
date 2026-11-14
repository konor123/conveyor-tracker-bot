"""FakeClock: ROS 시각과 steady 시각을 따로 조작한다."""


class FakeClock:
    def __init__(self, ros: float = 1000.0, steady: float = 50.0):
        self._ros = float(ros)
        self._steady = float(steady)
        self._ros_frozen = False

    def ros_now(self) -> float:
        return self._ros

    def steady_now(self) -> float:
        return self._steady

    def advance(self, dt: float) -> None:
        """두 시계를 함께 진행한다. ROS 시각이 멈춰 있으면 steady만 진행한다."""
        if dt < 0:
            raise ValueError("advance는 음수를 받지 않는다(jump_ros 사용)")
        self._steady += dt
        if not self._ros_frozen:
            self._ros += dt

    def advance_ros(self, dt: float) -> None:
        if dt < 0:
            raise ValueError("advance_ros는 음수를 받지 않는다(jump_ros 사용)")
        self._ros += dt

    def freeze_ros(self) -> None:
        self._ros_frozen = True

    def unfreeze_ros(self) -> None:
        self._ros_frozen = False

    def jump_ros(self, dt: float) -> None:
        """ROS 시각만 dt만큼 옮긴다(음수 = 역행)."""
        self._ros += dt
