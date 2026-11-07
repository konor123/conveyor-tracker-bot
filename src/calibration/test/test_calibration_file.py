import copy

import numpy as np
import pytest
import yaml

import synth
from calibration.calibration_file import (REQUIRED_KEYS, CalibrationFileError, load_extrinsics,
                                          make_extrinsics, save_extrinsics)

PROFILE = {
    'serial': '313522301601', 'firmware': '5.17.0.9', 'stream': 'color', 'width': 1280, 'height': 800,
    'fps': 30, 'fx': synth.K[0, 0], 'fy': synth.K[1, 1], 'cx': synth.K[0, 2], 'cy': synth.K[1, 2],
    'distortion_model': 'plumb_bob', 'distortion': synth.DIST.tolist(),
}


def _doc():
    return make_extrinsics(calibration_id='cal-2026-11-07-01', date='2026-11-07', T=synth.camera_pose(),
                           camera_profile=PROFILE, residuals={'method': 'pnp', 'n_points': 20, 'rms_px': 0.3},
                           parent_frame='base_link', child_frame='camera_color_optical_frame')


@pytest.mark.req("REQ-CAL-005")
@pytest.mark.tc("TC-CAL-005-01")
@pytest.mark.level("U")
def test_roundtrip(tmp_path):
    path = tmp_path / 'camera_extrinsics.yaml'
    save_extrinsics(_doc(), path)
    cal = load_extrinsics(path)
    assert cal.calibration_id == 'cal-2026-11-07-01'
    assert np.abs(cal.T - synth.camera_pose()).max() <= 1e-12
    assert np.abs(cal.K - synth.K).max() == 0.0
    assert np.abs(cal.dist - synth.DIST).max() == 0.0


@pytest.mark.req("REQ-CAL-005")
@pytest.mark.tc("TC-CAL-005-01")
@pytest.mark.level("U")
@pytest.mark.parametrize("key", REQUIRED_KEYS)
def test_missing_required_key_fails(tmp_path, key):
    doc = copy.deepcopy(_doc())
    del doc[key]
    path = tmp_path / 'bad.yaml'
    path.write_text(yaml.safe_dump(doc))
    with pytest.raises(CalibrationFileError) as e:
        load_extrinsics(path)
    assert key in str(e.value)


@pytest.mark.req("REQ-CAL-005")
@pytest.mark.tc("TC-CAL-005-01")
@pytest.mark.level("U")
@pytest.mark.parametrize("field", ['empty_id', 'empty_residuals', 'profile_fx', 'bad_rotation'])
def test_invalid_values_fail(tmp_path, field):
    doc = copy.deepcopy(_doc())
    if field == 'empty_id':
        doc['calibration_id'] = ''
    elif field == 'empty_residuals':
        doc['residuals'] = {}
    elif field == 'profile_fx':
        del doc['camera_profile']['fx']
    else:
        doc['transform']['rotation_xyzw'] = [0.0, 0.0, 0.0, 0.0]
    path = tmp_path / 'bad.yaml'
    path.write_text(yaml.safe_dump(doc))
    with pytest.raises(CalibrationFileError):
        load_extrinsics(path)
