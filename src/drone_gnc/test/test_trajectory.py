import numpy as np

from drone_gnc.trajectory import (
    MissionParameters,
    TrajectoryParameters,
    lemniscate_reference,
    mission_reference,
    smooth_lemniscate_reference,
)


def test_reference_at_zero_matches_specification():
    reference = lemniscate_reference(0.0)
    np.testing.assert_allclose(reference["position"], [0.0, 0.0, -2.0])
    np.testing.assert_allclose(reference["velocity"], [0.75, 1.0, 0.0])
    np.testing.assert_allclose(reference["acceleration"], [0.0, 0.0, -0.03125])
    np.testing.assert_allclose(reference["yaw"], np.arctan2(1.0, 0.75))


def test_reference_is_clamped_after_mission_duration():
    parameters = TrajectoryParameters(duration_s=60.0)
    at_end = lemniscate_reference(60.0, parameters)
    after_end = lemniscate_reference(80.0, parameters)
    np.testing.assert_allclose(at_end["state"], after_end["state"])


def test_mission_takes_off_to_figure8_start_then_aligns_yaw():
    parameters = MissionParameters()
    start = mission_reference(0.0, parameters)
    halfway = mission_reference(0.5 * parameters.takeoff_duration_s, parameters)
    alignment = mission_reference(parameters.takeoff_duration_s + 1.0, parameters)
    figure8_start = lemniscate_reference(0.0, parameters.trajectory)

    assert start["phase"] == "takeoff"
    np.testing.assert_allclose(start["position"], [0.0, 0.0, -0.035])
    np.testing.assert_allclose(start["velocity"], np.zeros(3), atol=1e-12)
    np.testing.assert_allclose(halfway["position"][0:2], np.zeros(2), atol=1e-12)
    assert figure8_start["position"][2] < halfway["position"][2] < -0.035
    assert alignment["phase"] == "yaw_align"
    np.testing.assert_allclose(alignment["position"], figure8_start["position"])
    np.testing.assert_allclose(alignment["velocity"], np.zeros(3), atol=1e-12)
    assert 0.0 < alignment["yaw"] < figure8_start["yaw"]


def test_mission_starts_figure8_with_continuous_position_velocity_and_acceleration():
    parameters = MissionParameters()
    figure8_start_s = (
        parameters.takeoff_duration_s
        + parameters.yaw_alignment_duration_s
    )
    just_before = mission_reference(figure8_start_s - 1e-6, parameters)
    mission_start = mission_reference(figure8_start_s, parameters)
    geometric_start = lemniscate_reference(0.0, parameters.trajectory)
    np.testing.assert_allclose(just_before["position"], geometric_start["position"])
    np.testing.assert_allclose(just_before["velocity"], np.zeros(3), atol=1e-12)
    np.testing.assert_allclose(mission_start["position"], geometric_start["position"])
    np.testing.assert_allclose(mission_start["velocity"], np.zeros(3), atol=1e-12)
    np.testing.assert_allclose(
        mission_start["acceleration"], np.zeros(3), atol=1e-12
    )
    np.testing.assert_allclose(mission_start["yaw"], geometric_start["yaw"])


def test_smooth_figure8_reaches_nominal_path_rate_without_changing_geometry():
    parameters = TrajectoryParameters()
    ramp_duration_s = 3.0
    ramp_end = smooth_lemniscate_reference(
        ramp_duration_s, parameters, ramp_duration_s
    )
    geometric = lemniscate_reference(0.5 * ramp_duration_s, parameters)
    np.testing.assert_allclose(ramp_end["position"], geometric["position"])
    np.testing.assert_allclose(ramp_end["velocity"], geometric["velocity"])
    np.testing.assert_allclose(ramp_end["acceleration"], geometric["acceleration"])
    assert ramp_end["path_time_rate"] == 1.0


def test_figure8_returns_to_start_after_common_period():
    parameters = TrajectoryParameters(duration_s=60.0)
    period_s = 2.0 * np.pi / parameters.omega_radps
    start = lemniscate_reference(0.0, parameters)
    end = lemniscate_reference(period_s, parameters)
    np.testing.assert_allclose(end["position"], start["position"], atol=1e-12)
    np.testing.assert_allclose(end["velocity"], start["velocity"], atol=1e-12)
