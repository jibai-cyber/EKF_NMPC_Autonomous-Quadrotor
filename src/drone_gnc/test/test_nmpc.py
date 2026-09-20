import numpy as np
import pytest

import drone_gnc.nmpc as nmpc_module
from drone_gnc.dynamics import VehicleParameters
from drone_gnc.nmpc import NmpcConfig, NmpcController


@pytest.mark.skipif(nmpc_module.ca is None, reason="CasADi is not installed")
def test_nmpc_solves_hover_problem():
    parameters = VehicleParameters()
    config = NmpcConfig(horizon_steps=3, step_s=0.05, ipopt_max_cpu_time_s=1.0)
    controller = NmpcController(parameters, config)
    state = np.zeros(13)
    state[2] = -2.5
    state[6] = 1.0
    references = np.tile(state, (config.horizon_steps + 1, 1))
    solution = controller.solve(state, references)
    assert solution["success"], solution["status"]
    assert solution["thrusts_n"].shape == (4,)
    assert np.all(solution["thrusts_n"] >= parameters.rotor_min_thrust_n)
    assert np.all(solution["thrusts_n"] <= parameters.rotor_max_thrust_n)
    np.testing.assert_allclose(
        solution["thrusts_n"], parameters.hover_thrust_per_rotor_n, atol=1e-3
    )


@pytest.mark.skipif(nmpc_module.ca is None, reason="CasADi is not installed")
def test_nmpc_enforces_slew_rate_throughout_prediction_horizon():
    parameters = VehicleParameters()
    config = NmpcConfig(
        horizon_steps=3,
        step_s=0.05,
        max_thrust_slew_nps=0.2,
        ipopt_max_cpu_time_s=1.0,
    )
    controller = NmpcController(parameters, config)
    state = np.zeros(13)
    state[2] = -2.0
    state[6] = 1.0
    references = np.tile(state, (config.horizon_steps + 1, 1))
    references[:, 2] = -5.0
    previous = np.full(4, parameters.hover_thrust_per_rotor_n)
    solution = controller.solve(state, references, previous)
    assert solution["success"], solution["status"]
    sequence = np.vstack((previous, solution["control_sequence"]))
    maximum_delta = config.max_thrust_slew_nps * config.step_s
    assert np.all(np.abs(np.diff(sequence, axis=0)) <= maximum_delta + 1e-6)


@pytest.mark.skipif(nmpc_module.ca is None, reason="CasADi is not installed")
def test_warm_start_shifts_previous_solution_by_one_step():
    parameters = VehicleParameters()
    config = NmpcConfig(horizon_steps=3, step_s=0.05)
    controller = NmpcController(parameters, config)
    state_count = controller.state_size * (config.horizon_steps + 1)
    old_states = np.arange(state_count, dtype=float).reshape(
        (controller.state_size, config.horizon_steps + 1), order="F"
    )
    old_states[6:10, :] = np.array([[1.0], [0.0], [0.0], [0.0]])
    old_controls = np.tile(
        np.arange(config.horizon_steps, dtype=float), (controller.input_size, 1)
    ) + parameters.hover_thrust_per_rotor_n
    controller._last_solution = np.concatenate(
        (
            old_states.reshape(-1, order="F"),
            old_controls.reshape(-1, order="F"),
        )
    )
    current = np.zeros(controller.state_size)
    current[6] = 1.0
    shifted = controller._shifted_warm_start(
        current, np.zeros_like(controller._last_solution)
    )
    shifted_states = shifted[:state_count].reshape(
        (controller.state_size, config.horizon_steps + 1), order="F"
    )
    shifted_controls = shifted[state_count:].reshape(
        (controller.input_size, config.horizon_steps), order="F"
    )
    np.testing.assert_allclose(shifted_states[:, 0], current)
    np.testing.assert_allclose(shifted_states[:, 1], old_states[:, 2])
    np.testing.assert_allclose(shifted_controls[:, 0], old_controls[:, 1])
    np.testing.assert_allclose(shifted_controls[:, -1], old_controls[:, -1])
