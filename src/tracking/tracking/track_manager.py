"""Frame-based cap tracking; no ROS, TF, transport, or wall-clock dependencies."""
from dataclasses import dataclass
import numpy as np
from .association import Det, associate
from .kalman import KalmanCV, measurement_covariance
from .speed_change import SpeedChangeDetector
from .belt_motion import estimate_belt_motion

OBSERVED, COASTING, LOST = 0, 1, 2


@dataclass(frozen=True)
class TrackedObject:
    track_id: int
    epoch: int
    status: int
    t_state: float
    position: np.ndarray
    velocity: np.ndarray
    state_cov: np.ndarray
    last_observed: float
    observation_count: int
    valid_until: float
    created_at: float
    object_class: int = 2


@dataclass(frozen=True)
class ResetResult:
    result: int
    new_epoch: int


@dataclass
class _Track:
    track_id: int
    kf: KalmanCV
    created_at: float
    last_observed: float
    observation_count: int = 1
    status: int = OBSERVED
    z_mean: float = 0.


class TrackManager:
    def __init__(self, *, sigma_accel_mps2=.01, meas_std_default_m=.0003,
                 init_vel_std_mps=.05, gate_chi2=100., coast_max_s=1.,
                 x_obs_min_m=-.35, x_obs_max_m=.35, exit_margin_m=.01,
                 cusum_k=1., cusum_h=8., recover_s=2., window_max_s=2.,
                 v_min_mps=.002, backstep_tol_s=0.):
        self.sigma_accel_mps2 = sigma_accel_mps2
        self.meas_std_default_m = meas_std_default_m
        self.init_vel_std_mps = init_vel_std_mps
        self.gate_chi2 = gate_chi2
        self.coast_max_s = coast_max_s
        self.x_obs_min_m, self.x_obs_max_m = x_obs_min_m, x_obs_max_m
        self.exit_margin_m = exit_margin_m
        self.window_max_s, self.v_min_mps = window_max_s, v_min_mps
        self.backstep_tol_s = backstep_tol_s
        self._detector_params = dict(cusum_k=cusum_k, cusum_h=cusum_h, recover_s=recover_s)
        self.detector = SpeedChangeDetector(**self._detector_params)
        self.epoch = 0
        self._next_id = 1
        self._tracks = []
        self.t_state = self._clock = None
        self.dropped_frames = 0
        self.belt_motion = None

    def _snapshot(self, track):
        return TrackedObject(track.track_id, self.epoch, track.status, track.kf.t_state,
                             np.r_[track.kf.x[:2], track.z_mean], np.r_[track.kf.x[2:], 0.],
                             track.kf.P.copy(), track.last_observed, track.observation_count,
                             track.last_observed+self.coast_max_s, track.created_at)

    @property
    def tracks(self):
        return [self._snapshot(t) for t in self._tracks]

    def reset(self, region=0):
        self._tracks.clear()
        self.epoch += 1
        self.t_state = None
        self.belt_motion = None
        self.detector = SpeedChangeDetector(**self._detector_params)
        return ResetResult(0, self.epoch)

    def on_clock(self, t_ros):
        backstep = self._clock is not None and self._clock-t_ros > self.backstep_tol_s
        self._clock = t_ros
        if backstep:
            self.reset()
        return backstep

    def process_frame(self, t_stamp, dets):
        if self.t_state is not None and t_stamp < self.t_state:
            self.dropped_frames += 1
            return []  # dropped frames cause no publication
        self.t_state = t_stamp
        valid = [Det(d.position, measurement_covariance(d.position_cov, self.meas_std_default_m))
                 for d in dets if d.object_class == 2 and d.position_valid
                 and d.position.size in (2, 3) and np.all(np.isfinite(d.position))]
        for track in self._tracks:
            track.kf.predict(t_stamp)
            # SPEC-300 4.2: past the exit or past the coast limit -> LOST before association,
            # so a detection after a frame gap never revives an expired track.
            if (track.kf.x[0] > self.x_obs_max_m+self.exit_margin_m
                    or t_stamp-track.last_observed > self.coast_max_s):
                track.status = LOST
        assignment = associate(self.tracks, valid, self.gate_chi2)
        # Select the source with post-assignment observation counts, matching BeltMotion.
        matched = {i for i, _ in assignment.matches}
        eligible = [i for i, t in enumerate(self._tracks)
                    if t.kf.x[0] <= self.x_obs_max_m+self.exit_margin_m
                    and (i in matched or t_stamp-t.last_observed <= self.coast_max_s)]
        source = max(eligible, key=lambda i: self._tracks[i].observation_count+(i in matched),
                     default=None)
        self.detector.advance(t_stamp)
        for i, j in assignment.matches:
            track, det = self._tracks[i], valid[j]
            if i == source:
                innovation = (det.position[0]-track.kf.x[0]) / np.sqrt(
                    track.kf.P[0, 0]+det.position_cov[0, 0])
                if self.detector.update(t_stamp, float(innovation)):
                    # Clear cross covariance when replacing the velocity prior, preserving PSD.
                    track.kf.P[:2, 2:] = track.kf.P[2:, :2] = 0.
                    track.kf.P[2:, 2:] = np.eye(2)*self.init_vel_std_mps**2
            track.kf.update(t_stamp, det.position[:2], det.position_cov)
            track.last_observed = t_stamp
            track.observation_count += 1
            if det.position.size == 3:
                track.z_mean += (det.position[2]-track.z_mean)/track.observation_count
            track.status = OBSERVED
        for i in assignment.unmatched_tracks:
            track = self._tracks[i]
            if track.status != LOST:
                track.status = COASTING if t_stamp-track.last_observed <= self.coast_max_s else LOST
        for j in assignment.unmatched_dets:
            det = valid[j]
            velocity = self.belt_motion.velocity if self.belt_motion is not None else np.zeros(2)
            p = np.zeros((4, 4))
            p[:2, :2] = det.position_cov
            p[2:, 2:] = np.eye(2)*self.init_vel_std_mps**2
            kf = KalmanCV(self.sigma_accel_mps2, np.r_[det.position[:2], velocity], p)
            kf.predict(t_stamp)
            self._tracks.append(_Track(self._next_id, kf, t_stamp, t_stamp,
                                       z_mean=float(det.position[2]) if det.position.size == 3 else 0.))
            self._next_id += 1
        for track in self._tracks:
            if track.kf.x[0] > self.x_obs_max_m+self.exit_margin_m:
                track.status = LOST
        output = self.tracks
        self.belt_motion = estimate_belt_motion(
            output, self.belt_motion, v_min_mps=self.v_min_mps, window_max_s=self.window_max_s,
            steady=self.detector.steady, last_change_stamp=self.detector.last_change_stamp)
        self._tracks = [t for t in self._tracks if t.status != LOST]
        return output
