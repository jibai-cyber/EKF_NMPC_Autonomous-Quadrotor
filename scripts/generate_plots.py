#!/usr/bin/env python3
"""Generate the core report plots from results/flight_log.csv."""

import argparse
from pathlib import Path

import matplotlib.pyplot as plt
import pandas as pd


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("csv", nargs="?", default="results/flight_log.csv")
    parser.add_argument("--output", default="results/plots")
    args = parser.parse_args()
    data = pd.read_csv(args.csv)
    output = Path(args.output)
    output.mkdir(parents=True, exist_ok=True)

    figure = plt.figure(figsize=(8, 6))
    axis = figure.add_subplot(111, projection="3d")
    axis.plot(data.reference_x, data.reference_y, data.reference_z, "k--", label="reference")
    axis.plot(data.estimate_x, data.estimate_y, data.estimate_z, label="estimate")
    axis.plot(data.truth_x, data.truth_y, data.truth_z, alpha=0.7, label="truth")
    axis.set(xlabel="North [m]", ylabel="East [m]", zlabel="Down [m]")
    axis.legend()
    figure.tight_layout()
    figure.savefig(output / "trajectory_3d.png", dpi=180)

    figure, axes = plt.subplots(3, 1, figsize=(9, 8), sharex=True)
    for axis, name in zip(axes, ("x", "y", "z")):
        error = data[f"estimate_{name}"] - data[f"truth_{name}"]
        bound = 2.0 * data[f"sigma_{name}"]
        axis.plot(data.time_s, error, label=f"{name} error")
        axis.fill_between(data.time_s, -bound, bound, alpha=0.25, label="+/- 2 sigma")
        axis.set_ylabel("Error [m]")
        axis.grid(True)
        axis.legend()
    axes[-1].set_xlabel("Time [s]")
    figure.tight_layout()
    figure.savefig(output / "ekf_position_error_2sigma.png", dpi=180)

    figure, axis = plt.subplots(figsize=(9, 4))
    for index in range(4):
        axis.plot(data.time_s, data[f"thrust_{index}_n"], label=f"T{index}")
    axis.axhline(0.2, color="k", linestyle="--")
    axis.axhline(5.5, color="k", linestyle="--")
    axis.set(xlabel="Time [s]", ylabel="Rotor thrust [N]")
    axis.grid(True)
    axis.legend(ncol=4)
    figure.tight_layout()
    figure.savefig(output / "rotor_thrusts.png", dpi=180)

    figure, axis = plt.subplots(figsize=(9, 4))
    axis.plot(data.time_s, data.solver_time_ms)
    axis.set(xlabel="Time [s]", ylabel="NMPC solve time [ms]")
    axis.grid(True)
    figure.tight_layout()
    figure.savefig(output / "solver_time.png", dpi=180)


if __name__ == "__main__":
    main()

