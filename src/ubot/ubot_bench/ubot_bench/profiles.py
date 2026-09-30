"""Sensor profiles (ubot_description/config/sensor_profiles.yaml) and their noise models.

The noise model is deliberately simple and fully specified by the datasheet numbers, so the
paper can state it in one equation:

    r_meas = r_true + b(r) + e,   e ~ N(0, sigma(r)^2),   b(r) ~ U(-B(r), B(r)) drawn once per run

sigma(r) is piecewise constant (sigma_bands), proportional (sigma_rel * r) or quadratic
(sigma_quad * r^2, stereo depth). B(r) is piecewise constant (bias_bands).
"""
import os

import numpy as np
import yaml

STACKS = {
    # stack id -> sensor profile (None = no range sensor used for localisation)
    'REF': 'lidar_ref_ust10lx',
    'MS200': 'lidar_ms200',
    'LD06': 'lidar_ld06',
    'A1': 'lidar_rplidar_a1',
    'OAKD': 'depth_oakd_lite',
    'ODOM': None,
}


def profiles_path():
    from ament_index_python.packages import get_package_share_directory
    return os.path.join(get_package_share_directory('ubot_description'), 'config',
                        'sensor_profiles.yaml')


def load_profiles(path=None):
    with open(path or profiles_path()) as f:
        return yaml.safe_load(f)


def _band_lookup(bands, r):
    """bands: [[r_upper, value], ...] ascending; value of the first band with r < r_upper."""
    uppers = np.array([b[0] for b in bands], dtype=float)
    values = np.array([b[1] for b in bands], dtype=float)
    idx = np.searchsorted(uppers, r, side='right')
    return values[np.minimum(idx, len(values) - 1)]


def sigma(profile, r):
    r = np.asarray(r, dtype=float)
    if 'sigma_bands' in profile:
        return _band_lookup(profile['sigma_bands'], r)
    if 'sigma_rel' in profile:
        return profile['sigma_rel'] * r
    if 'sigma_quad' in profile:
        return profile['sigma_quad'] * r * r
    raise KeyError(f"profile {profile.get('label')} has no sigma model")


def bias_bound(profile, r):
    return _band_lookup(profile.get('bias_bands', [[1e9, 0.0]]), np.asarray(r, dtype=float))


class RangeNoiseModel:
    """Applies the profile noise (and optional C3 degradation) to arrays of ranges.

    Degradation (C3 'degraded' condition, a dusty / dirty-window proxy):
      dropout   fraction of valid returns replaced by no-return (inf)
      outlier   fraction of valid returns replaced by a spurious short return U(range_min, r)
      noise_scale multiplies sigma(r)
    """

    def __init__(self, profile, seed, noise=True, dropout=0.0, outlier=0.0, noise_scale=1.0):
        self.p = profile
        self.rng = np.random.default_rng(seed)
        self.noise = noise
        self.dropout = dropout
        self.outlier = outlier
        self.noise_scale = noise_scale
        # One bias draw per run and per band (a calibration offset that stays for the run).
        bands = profile.get('bias_bands', [[1e9, 0.0]])
        self._bias_values = [self.rng.uniform(-b, b) for _, b in bands]
        self._bias_uppers = [u for u, _ in bands]

    def bias(self, r):
        return _band_lookup([[u, v] for u, v in zip(self._bias_uppers, self._bias_values)], r)

    def apply(self, ranges, range_min, range_max):
        r = np.asarray(ranges, dtype=np.float64).copy()
        valid = np.isfinite(r) & (r >= range_min) & (r <= range_max)
        if self.noise and valid.any():
            rv = r[valid]
            r[valid] = rv + self.bias(rv) + self.rng.normal(0.0, 1.0, rv.size) * sigma(self.p, rv) \
                * self.noise_scale
        if self.dropout > 0.0:
            drop = valid & (self.rng.random(r.size) < self.dropout)
            r[drop] = np.inf
            valid &= ~drop
        if self.outlier > 0.0:
            out = valid & (self.rng.random(r.size) < self.outlier)
            r[out] = self.rng.uniform(range_min, np.maximum(r[out], range_min + 1e-3))
        # Returns pushed outside the sensor's limits by noise are reported as no-return.
        r[np.isfinite(r) & ((r < range_min) | (r > range_max))] = np.inf
        return r.astype(np.float32)
