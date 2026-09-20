from dataclasses import fields

import numpy as np

from drone_gnc.ekf import EkfNoise, QuadrotorEkf


def test_noise_model_contains_only_white_noise_terms():
    assert [field.name for field in fields(EkfNoise)] == [
        "accel_std_mps2",
        "gyro_std_radps",
        "position_std_m",
    ]


def test_filter_state_and_covariance_have_13_elements():
    filter_ = QuadrotorEkf()
    filter_.initialize_position(np.array([0.0, 0.0, -2.5]))
    assert filter_.state.shape == (13,)
    assert filter_.covariance.shape == (13, 13)
    assert filter_.public_state.shape == (13,)
    assert filter_.public_covariance.shape == (13, 13)


def test_stationary_specific_force_remains_stationary():
    filter_ = QuadrotorEkf()
    filter_.initialize_position(np.array([0.0, 0.0, -2.5]))
    # At upright hover in FRD, an accelerometer measures -g along body z.
    filter_.predict(np.array([0.0, 0.0, -9.81]), np.zeros(3), 0.01)
    np.testing.assert_allclose(
        filter_.public_state[0:6], [0.0, 0.0, -2.5, 0.0, 0.0, 0.0], atol=1e-9
    )


def test_gnss_update_reduces_position_uncertainty():
    filter_ = QuadrotorEkf()
    filter_.initialize_position(np.zeros(3))
    before = np.diag(filter_.public_covariance)[0:3].copy()
    filter_.update_position(np.array([0.01, -0.01, -2.5]))
    after = np.diag(filter_.public_covariance)[0:3]
    assert np.all(after < before)
