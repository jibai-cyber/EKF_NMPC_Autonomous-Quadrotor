"""Tests for WGS84 geodetic-to-local-NED conversion."""

from drone_gnc.geodesy import (
    WGS84_SEMI_MAJOR_AXIS_M,
    Wgs84LocalFrame,
    wgs84_curvature_radii,
)

import numpy as np


REFERENCE_LATITUDE_DEG = 1.3521
REFERENCE_LONGITUDE_DEG = 103.8198


def test_reference_maps_to_ned_origin():
    """The configured geodetic reference is the local NED origin."""
    frame = Wgs84LocalFrame(
        REFERENCE_LATITUDE_DEG, REFERENCE_LONGITUDE_DEG, 0.0
    )
    np.testing.assert_allclose(
        frame.position_ned(
            REFERENCE_LATITUDE_DEG, REFERENCE_LONGITUDE_DEG, 0.0
        ),
        np.zeros(3),
        atol=1e-9,
    )


def test_wgs84_local_frame_has_correct_north_and_east_scale():
    """Local North and East use their respective WGS84 curvature radii."""
    latitude_rad = np.deg2rad(REFERENCE_LATITUDE_DEG)
    meridional_radius, prime_vertical_radius = wgs84_curvature_radii(
        latitude_rad
    )
    north_m = 3.0
    east_m = 2.0
    latitude_deg = REFERENCE_LATITUDE_DEG + np.rad2deg(
        north_m / meridional_radius
    )
    longitude_deg = REFERENCE_LONGITUDE_DEG + np.rad2deg(
        east_m / (prime_vertical_radius * np.cos(latitude_rad))
    )
    frame = Wgs84LocalFrame(
        REFERENCE_LATITUDE_DEG, REFERENCE_LONGITUDE_DEG, 0.0
    )
    position = frame.position_ned(latitude_deg, longitude_deg, 0.0)
    np.testing.assert_allclose(position[0:2], [north_m, east_m], atol=2e-6)
    assert abs(position[2]) < 2e-6


def test_altitude_increase_is_negative_down():
    """Increasing ellipsoid height produces a negative Down displacement."""
    frame = Wgs84LocalFrame(
        REFERENCE_LATITUDE_DEG, REFERENCE_LONGITUDE_DEG, 0.0
    )
    np.testing.assert_allclose(
        frame.position_ned(
            REFERENCE_LATITUDE_DEG, REFERENCE_LONGITUDE_DEG, 2.0
        ),
        [0.0, 0.0, -2.0],
        atol=1e-9,
    )


def test_previous_spherical_north_formula_has_the_observed_scale_error():
    """The removed spherical approximation reproduces the diagnosed error."""
    latitude_rad = np.deg2rad(REFERENCE_LATITUDE_DEG)
    meridional_radius, _ = wgs84_curvature_radii(latitude_rad)
    scale_error = WGS84_SEMI_MAJOR_AXIS_M / meridional_radius - 1.0
    np.testing.assert_allclose(scale_error, 0.00673387, rtol=1e-6)
