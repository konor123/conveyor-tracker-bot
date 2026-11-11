import pytest
from robot_control.sync_metrics import SyncCsv,summarize
from robot_control.belt_sync import BeltSynchronizer,PlanSample
pytestmark = [pytest.mark.level('U'),pytest.mark.req('REQ-ROB-013'),pytest.mark.tc('TC-ROB-013-05')]

def test_csv_retains_faults_and_refuses_overwrite(tmp_path):
    path=tmp_path/'run.csv'; w=SyncCsv(path)
    c=BeltSynchronizer(); c.arm(1,7,2)
    c.accept(PlanSample(1,7,2,1,1.,1.,2.,3.,(.52,0.),(.02,0.)),1.,10.)
    out=c.step(1.,10.,(.499,0.),1.,'robot_base')
    w.write(c,out,1.,10.,(.499,0.),1.)
    out=c.stop('TEST_STOP'); w.write(c,out,1.01,10.01,(.499,0.),1.01); w.close()
    result=summarize(path)
    assert result['samples']==2 and result['inactive_samples']==1
    assert result['xy_error_p95_m']==pytest.approx(.001)
    assert result['reasons']['TEST_STOP']==1
    with pytest.raises(FileExistsError): SyncCsv(path)

def test_empty_csv(tmp_path):
    path=tmp_path/'empty.csv'; w=SyncCsv(path); w.close()
    assert summarize(path)['xy_error_p95_m'] is None
