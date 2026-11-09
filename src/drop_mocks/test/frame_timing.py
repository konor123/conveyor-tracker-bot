"""Event-phase oracle shared by pure and ROS timing tests."""


def assert_event_phase(samples, start, end=float('inf'), frame=1 / 30):
    """Samples are (scenario frame time, event effect), never receive times."""
    active = [(t, effect) for t, effect in samples if start <= t < end]
    assert active and any(effect for _, effect in active), ('event not observed', start, end)
    for t, effect in samples:
        if t < start - 1e-6 or t >= end + frame - 1e-6:
            assert not effect, ('effect outside event', t, start, end)
        elif start + frame - 1e-6 <= t < end - 1e-6:
            assert effect, ('late or missing effect', t, start, end)
