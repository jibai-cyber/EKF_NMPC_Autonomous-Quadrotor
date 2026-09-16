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
