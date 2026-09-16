import numpy as np

from drone_gnc.dynamics import (
    VehicleParameters,
    condition_rotor_thrusts,
    continuous_dynamics,
    normalize_quaternion,
    rotor_wrench_frd,
    rk4_step,
)


def test_hover_acceleration_is_zero():
    parameters = VehicleParameters()
    state = np.zeros(13)
    state[6] = 1.0
    thrusts = np.full(4, parameters.hover_thrust_per_rotor_n)
    derivative = continuous_dynamics(state, thrusts, parameters)
    np.testing.assert_allclose(derivative, np.zeros(13), atol=1e-12)


def test_rotor_wrench_matches_document_mapping():
    parameters = VehicleParameters()
    thrusts = np.array([1.0, 2.0, 3.0, 4.0])
    force, torque = rotor_wrench_frd(thrusts, parameters)
    np.testing.assert_allclose(force, [0.0, 0.0, -10.0])
    np.testing.assert_allclose(
        torque,
        [parameters.dy_m * 4.0, 0.0, parameters.moment_ratio_m * 2.0],
    )


def test_rk4_preserves_unit_quaternion():
    state = np.zeros(13)
    state[6:10] = normalize_quaternion(np.array([1.0, 0.1, -0.2, 0.3]))
    state[10:13] = [0.2, -0.1, 0.4]
    parameters = VehicleParameters()
    result = rk4_step(
        state, np.full(4, parameters.hover_thrust_per_rotor_n), 0.01, parameters
    )
    np.testing.assert_allclose(np.linalg.norm(result[6:10]), 1.0, atol=1e-12)


def test_command_conditioner_limits_torque_and_slew():
    parameters = VehicleParameters()
    previous = np.full(4, parameters.hover_thrust_per_rotor_n)
    result = condition_rotor_thrusts(
        np.array([0.2, 5.5, 0.2, 5.5]),
        previous,
        0.05,
        parameters,
        max_roll_pitch_torque_nm=0.05,
        max_yaw_torque_nm=0.015,
        max_thrust_slew_nps=4.0,
    )
    _, torque = rotor_wrench_frd(result, parameters)
    assert np.all(np.abs(result - previous) <= 0.2 + 1e-12)
    assert abs(torque[0]) <= 0.05 + 1e-12
    assert abs(torque[1]) <= 0.05 + 1e-12
    assert abs(torque[2]) <= 0.015 + 1e-12
