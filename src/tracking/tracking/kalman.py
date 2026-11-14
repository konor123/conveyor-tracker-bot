"""Constant velocity filter in conveyor XY, with Joseph covariance updates."""
import numpy as np


def measurement_covariance(value, default_std=.0003):
    """Return a usable XY covariance; invalid/missing measurements use fallback."""
    try:
        r = np.asarray(value, dtype=float)
        if r.size == 9:
            r = r.reshape(3, 3)[:2, :2]
        elif r.size == 4:
            r = r.reshape(2, 2)
        if r.shape != (2, 2) or not np.all(np.isfinite(r)):
            raise ValueError('invalid covariance')
        if abs(r[0, 1]-r[1, 0]) > 1e-12 or r[0, 0] <= 0 or r[0, 0]*r[1, 1]-r[0, 1]**2 <= 0:
            raise ValueError('not positive definite')
        return r.copy()
    except (ValueError, TypeError, np.linalg.LinAlgError):
        return np.eye(2) * default_std**2


def inverse2(s):
    """Small SPD inverse avoids multithreaded BLAS dispatch for a 2x2 solve."""
    return np.array([[s[1, 1], -s[0, 1]], [-s[1, 0], s[0, 0]]]) / (
        s[0, 0]*s[1, 1]-s[0, 1]*s[1, 0])


class KalmanCV:
    """q_accel is acceleration standard deviation (m/s²), not variance.

    The constructor has no timestamp: the first predict/update anchors x0 to t.
    Subsequent calls with an earlier timestamp do not mutate the filter.
    """
    def __init__(self, q_accel, x0, P0):
        self.q_accel = float(q_accel)
        self.x = np.array(x0, dtype=float, copy=True)
        self.P = np.array(P0, dtype=float, copy=True)
        self.t_state = None

    def predict(self, t):
        if self.t_state is None:
            self.t_state = float(t)
            return
        dt = t - self.t_state
        if dt <= 0:
            return
        f = np.eye(4)
        f[:2, 2:] = np.eye(2) * dt
        g = np.vstack((np.eye(2) * (.5 * dt**2), np.eye(2) * dt))
        self.x = f @ self.x
        self.P = f @ self.P @ f.T + self.q_accel**2 * (g @ g.T)
        self.t_state = float(t)

    def update(self, t, z, R):
        if self.t_state is not None and t < self.t_state:
            return float('nan')
        self.predict(t)
        r = measurement_covariance(R)
        innovation = np.asarray(z)[:2] - self.x[:2]
        s = self.P[:2, :2] + r
        inverse = inverse2(s)
        k = self.P[:, :2] @ inverse
        nis = float(innovation @ inverse @ innovation)
        self.x += k @ innovation
        a = np.eye(4)
        a[:, :2] -= k
        self.P = a @ self.P @ a.T + k @ r @ k.T
        self.P = (self.P + self.P.T) * .5
        return nis
