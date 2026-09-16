#!/usr/bin/env python3
"""ROS-free dynamics smoke test retained as the requested top-level entry point."""

import numpy as np

from drone_gnc.dynamics import VehicleParameters, rk4_step


def main() -> None:
    parameters = VehicleParameters()
    state = np.zeros(13)
    state[2] = -2.5
    state[6] = 1.0
    thrusts = np.full(4, parameters.hover_thrust_per_rotor_n)
    for _ in range(1_000):
        state = rk4_step(state, thrusts, 0.01, parameters)
    print("Final hover state:", np.array2string(state, precision=6, suppress_small=True))


if __name__ == "__main__":
    main()

