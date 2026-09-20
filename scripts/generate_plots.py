#!/usr/bin/env python3
"""Generate rubric-oriented acceptance logs, metrics, and plots from a flight log."""

import argparse
from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np
import pandas as pd
import yaml


AXES = ("x", "y", "z")


def boolean_series(values: pd.Series) -> pd.Series:
    """Normalize booleans saved either as values or CSV strings."""
    if pd.api.types.is_bool_dtype(values):
        return values.fillna(False)
    return values.astype(str).str.strip().str.lower().isin({"1", "true", "yes"})


def finite_rows(data: pd.DataFrame, columns: list[str]) -> np.ndarray:
    """Return a mask selecting rows whose requested columns are all finite."""
    return np.isfinite(data[columns].to_numpy(dtype=float)).all(axis=1)


def quaternion_to_euler_deg(data: pd.DataFrame, prefix: str) -> tuple[np.ndarray, ...]:
    """Convert Hamilton body-to-world quaternions to roll, pitch, and yaw in degrees."""
    quaternion = data[
        [f"{prefix}_qw", f"{prefix}_qx", f"{prefix}_qy", f"{prefix}_qz"]
    ].to_numpy(dtype=float)
    norm = np.linalg.norm(quaternion, axis=1, keepdims=True)
    norm[norm < 1e-12] = np.nan
    quaternion = quaternion / norm
    qw, qx, qy, qz = quaternion.T
    roll = np.arctan2(2.0 * (qw * qx + qy * qz), 1.0 - 2.0 * (qx**2 + qy**2))
    pitch = np.arcsin(np.clip(2.0 * (qw * qy - qz * qx), -1.0, 1.0))
    yaw = np.arctan2(2.0 * (qw * qz + qx * qy), 1.0 - 2.0 * (qy**2 + qz**2))
    return tuple(np.rad2deg(angle) for angle in (roll, pitch, yaw))


def wrapped_angle_error_deg(actual: np.ndarray, reference: np.ndarray) -> np.ndarray:
    """Return the signed shortest angular difference in degrees."""
    return (actual - reference + 180.0) % 360.0 - 180.0


def add_derived_columns(data: pd.DataFrame, configuration: dict) -> pd.DataFrame:
    """Add errors, mission phase, attitude, torque, and constraint diagnostics."""
    data = data.copy()
    trajectory = configuration["trajectory_node"]["ros__parameters"]
    controller = configuration["nmpc_node"]["ros__parameters"]
    takeoff_end = float(trajectory["takeoff_duration_s"])
    figure8_start = takeoff_end + float(trajectory["yaw_alignment_duration_s"])
    figure8_end = (
        figure8_start
        + float(trajectory["duration_s"])
        + 0.5 * float(trajectory.get("figure8_entry_ramp_duration_s", 0.0))
    )
    phase_time = (
        data.reference_time_s
        if "reference_time_s" in data.columns
        else data.time_s
    )

    data["mission_phase"] = np.select(
        [phase_time < takeoff_end, phase_time < figure8_start, phase_time <= figure8_end],
        ["takeoff", "yaw_align", "figure8"],
        default="hold",
    )
    for axis in AXES:
        data[f"tracking_error_{axis}_m"] = data[f"truth_{axis}"] - data[f"reference_{axis}"]
        data[f"estimation_error_{axis}_m"] = data[f"estimate_{axis}"] - data[f"truth_{axis}"]
        data[f"velocity_tracking_error_{axis}_mps"] = (
            data[f"truth_v{axis}"] - data[f"reference_v{axis}"]
        )

    for prefix in ("truth", "estimate", "reference"):
        roll, pitch, yaw = quaternion_to_euler_deg(data, prefix)
        data[f"{prefix}_roll_deg"] = roll
        data[f"{prefix}_pitch_deg"] = pitch
        data[f"{prefix}_yaw_deg"] = yaw
    data["yaw_tracking_error_deg"] = wrapped_angle_error_deg(
        data.truth_yaw_deg.to_numpy(), data.reference_yaw_deg.to_numpy()
    )
    data["truth_tilt_deg"] = np.maximum(
        np.abs(data.truth_roll_deg), np.abs(data.truth_pitch_deg)
    )

    thrusts = data[[f"thrust_{index}_n" for index in range(4)]].to_numpy(dtype=float)
    t0, t1, t2, t3 = thrusts.T
    data["torque_x_nm"] = float(controller["dy_m"]) * (-t0 - t1 + t2 + t3)
    data["torque_y_nm"] = float(controller["dx_m"]) * (-t0 + t1 + t2 - t3)
    data["torque_z_nm"] = float(controller["moment_ratio_m"]) * (-t0 + t1 - t2 + t3)
    return data


def save_figure(figure: plt.Figure, path: Path) -> None:
    """Save and close a report figure."""
    figure.tight_layout()
    figure.savefig(path, dpi=180)
    plt.close(figure)


def generate_plots(data: pd.DataFrame, output: Path, configuration: dict) -> None:
    """Create trajectory, estimation, tracking, constraint, attitude, and timing plots."""
    output.mkdir(parents=True, exist_ok=True)
    trajectory = configuration["trajectory_node"]["ros__parameters"]
    controller = configuration["nmpc_node"]["ros__parameters"]
    takeoff_end = float(trajectory["takeoff_duration_s"])
    figure8_start = takeoff_end + float(trajectory["yaw_alignment_duration_s"])
    figure8_end = (
        figure8_start
        + float(trajectory["duration_s"])
        + 0.5 * float(trajectory.get("figure8_entry_ramp_duration_s", 0.0))
    )
    phase_time = (
        data.reference_time_s
        if "reference_time_s" in data.columns
        else data.time_s
    )
    mission_data = data.loc[phase_time <= figure8_end]
    figure8_data = data.loc[data.mission_phase == "figure8"]

    figure = plt.figure(figsize=(9, 7))
    axis = figure.add_subplot(111, projection="3d")
    axis.plot(
        mission_data.reference_x,
        mission_data.reference_y,
        mission_data.reference_z,
        "k--",
        label="reference",
    )
    axis.plot(
        mission_data.estimate_x,
        mission_data.estimate_y,
        mission_data.estimate_z,
        label="estimate",
    )
    axis.plot(
        mission_data.truth_x,
        mission_data.truth_y,
        mission_data.truth_z,
        alpha=0.8,
        label="truth",
    )
    axis.set(
        xlabel="North [m]",
        ylabel="East [m]",
        zlabel="Down [m]",
        title="Ground Takeoff, Yaw Alignment, and 3D Figure-8 Tracking",
    )
    axis.legend()
    save_figure(figure, output / "trajectory_3d.png")

    figure, axes = plt.subplots(3, 1, figsize=(10, 8), sharex=True)
    for axis, name in zip(axes, AXES):
        error = figure8_data[f"estimate_{name}"] - figure8_data[f"truth_{name}"]
        bound = 2.0 * figure8_data[f"sigma_{name}"]
        axis.plot(figure8_data.time_s, error, label=f"{name} estimation error")
        axis.fill_between(
            figure8_data.time_s, -bound, bound, alpha=0.25, label="±2σ"
        )
        axis.axhline(0.0, color="k", linewidth=0.7)
        axis.set_ylabel("Error [m]")
        axis.grid(True)
        axis.legend(loc="upper right")
    axes[0].set_title("Figure-8 EKF Position Estimation Error and ±2σ Bounds")
    axes[-1].set_xlabel("Simulation time [s]")
    save_figure(figure, output / "ekf_position_error_2sigma.png")

    figure, axes = plt.subplots(3, 1, figsize=(10, 8), sharex=True)
    for axis, name in zip(axes, AXES):
        axis.plot(
            figure8_data.time_s,
            figure8_data[f"tracking_error_{name}_m"],
            label=f"{name} truth-reference",
        )
        axis.axhline(0.0, color="k", linewidth=0.7)
        axis.set_ylabel("Error [m]")
        axis.grid(True)
        axis.legend(loc="upper right")
    axes[0].set_title("Figure-8 Position Tracking Errors")
    axes[-1].set_xlabel("Simulation time [s]")
    save_figure(figure, output / "tracking_errors.png")

    thrust_columns = [f"thrust_{index}_n" for index in range(4)]
    figure, axis = plt.subplots(figsize=(10, 4.5))
    for index, column in enumerate(thrust_columns):
        axis.plot(data.time_s, data[column], label=f"T{index}")
    axis.axhline(float(controller["rotor_min_thrust_n"]), color="k", linestyle="--", label="limits")
    axis.axhline(float(controller["rotor_max_thrust_n"]), color="k", linestyle="--")
    axis.set(
        xlabel="Simulation time [s]",
        ylabel="Rotor thrust [N]",
        title="Rotor Thrust Histories and Bounds",
    )
    axis.grid(True)
    axis.legend(ncol=5)
    save_figure(figure, output / "rotor_thrusts.png")

    figure, axes = plt.subplots(2, 1, figsize=(10, 8), sharex=True)
    for index, column in enumerate(thrust_columns):
        axes[0].plot(figure8_data.time_s, figure8_data[column], label=f"T{index}")
    axes[0].axhline(float(controller["rotor_min_thrust_n"]), color="k", linestyle="--")
    axes[0].axhline(float(controller["rotor_max_thrust_n"]), color="k", linestyle="--")
    axes[0].set_ylabel("Thrust [N]")
    axes[0].set_title("Control Input Constraint Verification")
    axes[0].grid(True)
    axes[0].legend(ncol=4)
    torque_limits = (
        float(controller["max_roll_pitch_torque_nm"]),
        float(controller["max_roll_pitch_torque_nm"]),
        float(controller["max_yaw_torque_nm"]),
    )
    for name, limit in zip(("x", "y", "z"), torque_limits):
        axes[1].plot(
            figure8_data.time_s,
            figure8_data[f"torque_{name}_nm"],
            label=f"τ{name}",
        )
        axes[1].axhline(limit, color="0.5", linestyle=":", linewidth=0.8)
        axes[1].axhline(-limit, color="0.5", linestyle=":", linewidth=0.8)
    axes[1].set(xlabel="Simulation time [s]", ylabel="Body torque [N·m]")
    axes[1].grid(True)
    axes[1].legend(ncol=3)
    save_figure(figure, output / "control_constraints.png")

    figure, axes = plt.subplots(2, 1, figsize=(10, 8), sharex=True)
    for name in ("roll", "pitch", "yaw"):
        axes[0].plot(
            figure8_data.time_s, figure8_data[f"truth_{name}_deg"], label=name
        )
    tilt_limit = float(controller["max_tilt_deg"])
    axes[0].axhline(tilt_limit, color="k", linestyle="--", label="roll/pitch limit")
    axes[0].axhline(-tilt_limit, color="k", linestyle="--")
    axes[0].set(ylabel="Angle [deg]", title="Truth Attitude and Body Rates")
    axes[0].grid(True)
    axes[0].legend(ncol=4)
    for name in ("p", "q", "r"):
        axes[1].plot(
            figure8_data.time_s,
            np.rad2deg(figure8_data[f"truth_{name}"]),
            label=name,
        )
    axes[1].set(xlabel="Simulation time [s]", ylabel="Rate [deg/s]")
    axes[1].grid(True)
    axes[1].legend(ncol=3)
    save_figure(figure, output / "attitude_rates.png")

    figure, axis = plt.subplots(figsize=(10, 4.5))
    nmpc_mask = data.mission_phase == "figure8"
    axis.plot(
        data.loc[nmpc_mask, "time_s"],
        data.loc[nmpc_mask, "solver_time_ms"],
        label="solve time",
    )
    control_deadline_ms = 1000.0 / float(controller["control_rate_hz"])
    axis.axhline(control_deadline_ms, color="tab:orange", linestyle="--", label="control period")
    axis.axhline(
        1000.0 * float(controller["ipopt_max_cpu_time_s"]),
        color="tab:red",
        linestyle=":",
        label="IPOPT CPU limit",
    )
    failed = nmpc_mask & ~boolean_series(data.solver_success)
    axis.scatter(
        data.loc[failed, "time_s"],
        data.loc[failed, "solver_time_ms"],
        color="red",
        marker="x",
        label="unsuccessful",
        zorder=3,
    )
    axis.set(
        xlabel="Simulation time [s]",
        ylabel="NMPC solve time [ms]",
        title="NMPC Computation Time and Real-Time Boundaries",
    )
    axis.grid(True)
    axis.legend()
    save_figure(figure, output / "solver_time.png")


def generate_acceptance_outputs(
    data: pd.DataFrame, results_dir: Path, configuration: dict
) -> None:
    """Write a concise human-readable log and long-form metric summary."""
    results_dir.mkdir(parents=True, exist_ok=True)
    controller = configuration["nmpc_node"]["ros__parameters"]
    figure8 = data.mission_phase == "figure8"
    metrics: list[dict[str, str | float | int]] = []

    def add(category: str, metric: str, value, unit: str = "") -> None:
        metrics.append({"category": category, "metric": metric, "value": value, "unit": unit})

    add("configuration", "imu_bias_model", "disabled_assumed_zero")
    add("mission", "logged_samples", int(len(data)), "samples")
    add("mission", "final_simulation_time", float(data.time_s.max()), "s")

    tracking_columns = [f"tracking_error_{axis}_m" for axis in AXES]
    tracking_mask = figure8 & finite_rows(data, tracking_columns)
    tracking = data.loc[tracking_mask, tracking_columns].to_numpy(dtype=float)
    if tracking.size:
        for index, axis in enumerate(AXES):
            add("tracking", f"figure8_{axis}_rmse", float(np.sqrt(np.mean(tracking[:, index] ** 2))), "m")
            add("tracking", f"figure8_{axis}_max_abs", float(np.max(np.abs(tracking[:, index]))), "m")
        norm = np.linalg.norm(tracking, axis=1)
        add("tracking", "figure8_3d_rmse", float(np.sqrt(np.mean(norm**2))), "m")
        add("tracking", "figure8_3d_max", float(np.max(norm)), "m")
        yaw_error = data.loc[tracking_mask, "yaw_tracking_error_deg"].to_numpy(dtype=float)
        add("tracking", "figure8_yaw_rmse", float(np.sqrt(np.nanmean(yaw_error**2))), "deg")

    estimation_columns = [f"estimation_error_{axis}_m" for axis in AXES]
    estimation_mask = figure8 & finite_rows(data, estimation_columns)
    estimation = data.loc[estimation_mask, estimation_columns].to_numpy(dtype=float)
    if estimation.size:
        for index, axis in enumerate(AXES):
            error = estimation[:, index]
            sigma = data.loc[estimation_mask, f"sigma_{axis}"].to_numpy(dtype=float)
            add("ekf", f"position_{axis}_rmse", float(np.sqrt(np.mean(error**2))), "m")
            add(
                "ekf",
                f"position_{axis}_2sigma_coverage",
                float(np.mean(np.abs(error) <= 2.0 * sigma) * 100.0),
                "%",
            )

    thrust_columns = [f"thrust_{index}_n" for index in range(4)]
    control_mask = figure8 & finite_rows(data, thrust_columns)
    thrust = data.loc[control_mask, thrust_columns].to_numpy(dtype=float)
    if thrust.size:
        thrust_min = float(controller["rotor_min_thrust_n"])
        thrust_max = float(controller["rotor_max_thrust_n"])
        add("constraints", "minimum_rotor_thrust", float(np.min(thrust)), "N")
        add("constraints", "maximum_rotor_thrust", float(np.max(thrust)), "N")
        violations = (thrust < thrust_min - 1e-9) | (thrust > thrust_max + 1e-9)
        add("constraints", "rotor_thrust_violation_count", int(np.sum(violations)), "samples")
        applied_steps = np.abs(np.diff(thrust, axis=0))
        maximum_step = (
            float(np.max(applied_steps)) if applied_steps.size else 0.0
        )
        step_limit = float(controller["max_thrust_slew_nps"]) / float(
            controller["control_rate_hz"]
        )
        add("constraints", "maximum_applied_thrust_step", maximum_step, "N/update")
        add("constraints", "thrust_step_limit", step_limit, "N/update")
        add(
            "constraints",
            "thrust_slew_violation_count",
            int(np.sum(applied_steps > step_limit + 1e-9)),
            "samples",
        )

    torque_limits = {
        "x": float(controller["max_roll_pitch_torque_nm"]),
        "y": float(controller["max_roll_pitch_torque_nm"]),
        "z": float(controller["max_yaw_torque_nm"]),
    }
    for axis, limit in torque_limits.items():
        values = data.loc[figure8, f"torque_{axis}_nm"].dropna().to_numpy(dtype=float)
        if values.size:
            add("constraints", f"maximum_abs_torque_{axis}", float(np.max(np.abs(values))), "N m")
            add(
                "constraints",
                f"torque_{axis}_violation_count",
                int(np.sum(np.abs(values) > limit + 1e-9)),
                "samples",
            )

    tilt = data.loc[figure8, "truth_tilt_deg"].dropna().to_numpy(dtype=float)
    if tilt.size:
        tilt_limit = float(controller["max_tilt_deg"])
        add("constraints", "maximum_truth_tilt", float(np.max(tilt)), "deg")
        add("constraints", "tilt_violation_count", int(np.sum(tilt > tilt_limit + 1e-9)), "samples")

    rate_limits = np.asarray(controller["max_body_rates_degps"], dtype=float)
    for index, axis in enumerate(("p", "q", "r")):
        values = np.abs(
            np.rad2deg(data.loc[figure8, f"truth_{axis}"].dropna().to_numpy(dtype=float))
        )
        if values.size:
            add("constraints", f"maximum_abs_body_rate_{axis}", float(np.max(values)), "deg/s")
            add(
                "constraints",
                f"body_rate_{axis}_violation_count",
                int(np.sum(values > rate_limits[index] + 1e-9)),
                "samples",
            )

    solver_mask = figure8 & np.isfinite(data.solver_time_ms.to_numpy(dtype=float))
    solve_time = data.loc[solver_mask, "solver_time_ms"].to_numpy(dtype=float)
    if solve_time.size:
        success = boolean_series(data.loc[solver_mask, "solver_success"]).to_numpy()
        deadline_ms = 1000.0 / float(controller["control_rate_hz"])
        add("solver", "success_rate", float(np.mean(success) * 100.0), "%")
        add("solver", "median_solve_time", float(np.median(solve_time)), "ms")
        add("solver", "p95_solve_time", float(np.percentile(solve_time, 95.0)), "ms")
        add("solver", "maximum_solve_time", float(np.max(solve_time)), "ms")
        add("solver", "control_deadline_miss_count", int(np.sum(solve_time > deadline_ms)), "samples")
        add("solver", "unsuccessful_solve_count", int(np.sum(~success)), "samples")

    metrics_frame = pd.DataFrame(metrics)
    metrics_frame.to_csv(results_dir / "acceptance_summary.csv", index=False)
    with (results_dir / "acceptance_summary.txt").open("w", encoding="utf-8") as stream:
        stream.write("MA6224 EKF-NMPC acceptance summary\n")
        stream.write("Generated from results/flight_log.csv\n")
        stream.write("Tracking metrics use the figure-8 phase only.\n\n")
        current_category = None
        for row in metrics:
            if row["category"] != current_category:
                current_category = row["category"]
                stream.write(f"[{current_category}]\n")
            value = row["value"]
            formatted = f"{value:.6g}" if isinstance(value, float) else str(value)
            suffix = f" {row['unit']}" if row["unit"] else ""
            stream.write(f"{row['metric']}: {formatted}{suffix}\n")
        stream.write("\nNote: the rubric defines qualitative bands, not numerical pass thresholds.\n")

    selected_columns = [
        "time_s",
        "mission_phase",
        *[f"reference_{axis}" for axis in AXES],
        *[f"truth_{axis}" for axis in AXES],
        *[f"estimate_{axis}" for axis in AXES],
        *tracking_columns,
        *estimation_columns,
        "truth_roll_deg",
        "truth_pitch_deg",
        "truth_yaw_deg",
        "reference_yaw_deg",
        "yaw_tracking_error_deg",
        *thrust_columns,
        "torque_x_nm",
        "torque_y_nm",
        "torque_z_nm",
        "solver_time_ms",
        "solver_iterations",
        "solver_success",
        "solver_status",
    ]
    available_columns = [column for column in selected_columns if column in data.columns]
    data[available_columns].to_csv(
        results_dir / "acceptance_log.csv", index=False, float_format="%.8g"
    )


def main() -> None:
    """Parse arguments and produce all acceptance artifacts."""
    parser = argparse.ArgumentParser()
    parser.add_argument("csv", nargs="?", default="results/flight_log.csv")
    parser.add_argument("--output", default="results/plots")
    parser.add_argument("--results-dir", default="results")
    parser.add_argument("--config", default="src/drone_gnc/config/project.yaml")
    args = parser.parse_args()

    data = pd.read_csv(args.csv)
    with Path(args.config).open(encoding="utf-8") as stream:
        configuration = yaml.safe_load(stream)
    if "solver_status" not in data:
        data["solver_status"] = "not_recorded"
    data = add_derived_columns(data, configuration)
    generate_plots(data, Path(args.output), configuration)
    generate_acceptance_outputs(data, Path(args.results_dir), configuration)


if __name__ == "__main__":
    main()
