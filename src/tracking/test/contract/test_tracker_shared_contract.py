"""Run the exact common tracker contract against the real node with a DDS feeder."""
import sys
from pathlib import Path
import pytest
pytest.importorskip('rclpy')
sys.path.insert(0,str(Path(__file__).resolve().parents[4] / 'tools/testing/contracts'))
import tracker_contract as shared
from tracker_contract import TestTrackerContract
from drop_ros import Target, to_time
from drop_interfaces.msg import ObjectDetection, ObjectDetectionArray
from geometry_msgs.msg import Point
from std_msgs.msg import Header



@pytest.fixture(scope='module')
def tracker_target():
    return Target(cmd=[sys.executable,'-c',
        'import rclpy; from tracking.tracker_node import TrackerNode; '
        'rclpy.init(); n=TrackerNode(); n.trigger_configure(); n.trigger_activate(); rclpy.spin(n)'],
        node='tracker_node')


@pytest.fixture(scope='module')
def tracker_contract(tracker_target):
    source = shared.tracker_contract.__wrapped__(tracker_target)
    h = next(source)
    pub = h.client.create_publisher(ObjectDetectionArray,'objects/detections',5)
    start = h.now()
    def feed():
        t = h.now()
        x = -.25+.02*((t-start)%15.)
        pub.publish(ObjectDetectionArray(header=Header(stamp=to_time(t),frame_id='robot_base'),
            detections=[ObjectDetection(object_class=2,position_valid=True,
                position=Point(x=x),position_cov=[9e-8,0.,0.,0.,9e-8,0.,0.,0.,9e-8])]))
    timer = h.client.create_timer(1/30.,feed)
    h.wait_for(lambda: len(h.tracked) >= 5 and len(h.belt) >= 5,8.)
    try:
        yield h
    finally:
        h.client.destroy_timer(timer)
        try:
            next(source)
        except StopIteration:
            pass
