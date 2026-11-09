"""구독·발행 생성 시 QoS 이벤트 콜백 설정(SPEC-010 6절).

rclpy는 구독·발행마다 기본 QoS 이벤트 핸들러(incompatible QoS 경고)를 waitable로 붙인다. MultiThreadedExecutor는
이벤트마다 대기 집합을 다시 만들며 이 waitable을 모두 순회하므로, 30 Hz 토픽이 여럿인 노드에서 CPU의 대부분을
쓰고 콜백 지연(heartbeat 오검출)을 만든다. QoS 불일치는 계약 시험(`check_qos_compatible`)으로 검출하므로
실행 노드에서는 기본 핸들러를 붙이지 않는다.
"""
from rclpy.event_handler import PublisherEventCallbacks, SubscriptionEventCallbacks


def no_sub_events() -> SubscriptionEventCallbacks:
    return SubscriptionEventCallbacks(use_default_callbacks=False)


def no_pub_events() -> PublisherEventCallbacks:
    return PublisherEventCallbacks(use_default_callbacks=False)
