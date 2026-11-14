import os

import cv2
import numpy as np
import pytest
import yaml

import synth
from calibration.calibration_file import load_extrinsics
from calibration.charuco import board_points
from calibration.cli import calibrate_camera
from test_board_session import SPEC, TOUCH_IDS, _board_pose
from test_charuco import _render

BOARD = {'squares_x_count': 5, 'squares_y_count': 7, 'square_m': 0.030, 'marker_m': 0.022,
         'dictionary': 'DICT_4X4_50', 'board_thickness_m': 0.0}
PROFILE = {'serial': 'synthetic', 'width': synth.W, 'height': synth.H, 'fx': float(synth.K[0, 0]),
           'fy': float(synth.K[1, 1]), 'cx': float(synth.K[0, 2]), 'cy': float(synth.K[1, 2]),
           'distortion_model': 'plumb_bob', 'distortion': [0.0] * 5}


def test_calibrate_camera_end_to_end(tmp_path):
    rng = np.random.default_rng(3)
    T_cam = synth.camera_pose()
    views = []
    plan = [(synth.BELT_Z, 'belt', 'fit')] * 4 + [(synth.MAT_Z, 'mat', 'fit')] * 4 + \
           [(synth.SPACER_Z, 'spacer', 'fit')] * 4 + [(synth.BELT_Z, 'belt', 'verify')] * 5
    for k, (z, surface, role) in enumerate(plan):
        T_bb = _board_pose(rng, z)
        img = _render(synth.inv(T_cam) @ T_bb, synth.K)
        name = f'{surface}-{role}-{k}'
        cv2.imwrite(str(tmp_path / f'{name}.png'), img)
        touch = synth.transform(T_bb, board_points(SPEC, TOUCH_IDS)) + rng.normal(0, 0.0002, (4, 3))
        views.append({'name': name, 'image': f'{name}.png', 'role': role, 'surface': surface,
                      'touches_m': {int(i): p.tolist() for i, p in zip(TOUCH_IDS, touch)}})
    (tmp_path / 'board.yaml').write_text(yaml.safe_dump(BOARD))
    (tmp_path / 'profile.yaml').write_text(yaml.safe_dump(PROFILE))
    session = {'calibration_id': 'cal-test-01', 'date': '2026-11-07', 'board': 'board.yaml',
               'camera_profile': 'profile.yaml', 'views': views}
    (tmp_path / 'session.yaml').write_text(yaml.safe_dump(session))
    out = tmp_path / 'out'
    assert calibrate_camera([str(tmp_path / 'session.yaml'), '--out', str(out)]) == 0
    cal = load_extrinsics(out / 'camera_extrinsics.yaml')
    assert cal.calibration_id == 'cal-test-01'
    assert cal.residuals['verify']['pass'] is True
    assert np.degrees(synth.rot_err_rad(cal.T[:3, :3], T_cam[:3, :3])) <= 0.2
    for name in ('belt_plane.yaml', 'mat_plane.yaml', 'spacer_plane.yaml', 'calib-verify-2026-11-07.csv',
                 'report.yaml'):
        assert os.path.exists(out / name)
    belt = yaml.safe_load((out / 'belt_plane.yaml').read_text())
    assert belt['calibration_id'] == 'cal-test-01'
    assert belt['offset_m'] == pytest.approx(synth.BELT_Z, abs=0.0005)
