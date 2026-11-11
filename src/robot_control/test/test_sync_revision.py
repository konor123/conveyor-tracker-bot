from dataclasses import replace
import pytest
from robot_control.belt_sync import BeltSynchronizer, PlanSample, SyncConfig
pytestmark = [pytest.mark.level('U'), pytest.mark.req('REQ-ROB-013'), pytest.mark.tc('TC-ROB-013-02')]

def test_plan_and_release_uncertainty_are_separate():
    c = BeltSynchronizer(); c.arm(1,7,2)
    p = PlanSample(1,7,2,1,1.,1.,2.,3.,(.52,0.),(.02,0.),position_std_m=.002)
    assert c.accept(p,1.,10.)
    assert not c.release_uncertainty_ok(1.,10.)
    assert c.accept(replace(p,revision=2,position_std_m=.0005),1.,10.)
    assert c.release_uncertainty_ok(1.,10.)
    assert not c.release_uncertainty_ok(1.3,10.3)
    assert not c.release_uncertainty_ok(1.,10.3)
    c.stop(); assert not c.release_uncertainty_ok(1.,10.)

@pytest.mark.parametrize('std,accepted',[(.00325,True),(.003251,False)])
def test_plan_boundary(std,accepted):
    c=BeltSynchronizer(); c.arm(1,7,2)
    assert c.accept(PlanSample(1,7,2,1,1.,1.,2.,3.,(.52,0.),(.02,0.),position_std_m=std),1.,10.) is accepted

def test_invalid_release_limit():
    with pytest.raises(ValueError): SyncConfig(release_position_std_max_m=.004)
