"""WGS84 geodetic coordinates and a local North-East-Down frame."""

import numpy as np


WGS84_SEMI_MAJOR_AXIS_M = 6_378_137.0
WGS84_INVERSE_FLATTENING = 298.257223563
WGS84_FLATTENING = 1.0 / WGS84_INVERSE_FLATTENING
WGS84_ECCENTRICITY_SQUARED = WGS84_FLATTENING * (2.0 - WGS84_FLATTENING)


def wgs84_curvature_radii(latitude_rad: float) -> tuple[float, float]:
    """Return meridional and prime-vertical radii at a geodetic latitude."""
    sine = np.sin(float(latitude_rad))
    denominator = np.sqrt(1.0 - WGS84_ECCENTRICITY_SQUARED * sine * sine)
    prime_vertical = WGS84_SEMI_MAJOR_AXIS_M / denominator
    meridional = (
        WGS84_SEMI_MAJOR_AXIS_M
        * (1.0 - WGS84_ECCENTRICITY_SQUARED)
        / denominator**3
    )
    return float(meridional), float(prime_vertical)


def geodetic_to_ecef(
    latitude_deg: float, longitude_deg: float, altitude_m: float
) -> np.ndarray:
    """Convert WGS84 geodetic latitude, longitude and altitude to ECEF."""
    latitude = np.deg2rad(float(latitude_deg))
    longitude = np.deg2rad(float(longitude_deg))
    sine_latitude = np.sin(latitude)
    cosine_latitude = np.cos(latitude)
    sine_longitude = np.sin(longitude)
    cosine_longitude = np.cos(longitude)
    _, prime_vertical = wgs84_curvature_radii(latitude)
    radial = prime_vertical + float(altitude_m)
    return np.array(
        [
            radial * cosine_latitude * cosine_longitude,
            radial * cosine_latitude * sine_longitude,
            (
                prime_vertical * (1.0 - WGS84_ECCENTRICITY_SQUARED)
                + float(altitude_m)
            )
            * sine_latitude,
        ]
    )


class Wgs84LocalFrame:
    """Project WGS84 geodetic positions into a reference-centred NED frame."""

    def __init__(
        self,
        reference_latitude_deg: float,
        reference_longitude_deg: float,
        reference_altitude_m: float,
    ) -> None:
        """Precompute the reference ECEF origin and ECEF-to-NED rotation."""
        self.reference_latitude_deg = float(reference_latitude_deg)
        self.reference_longitude_deg = float(reference_longitude_deg)
        self.reference_altitude_m = float(reference_altitude_m)
        self._reference_ecef = geodetic_to_ecef(
            self.reference_latitude_deg,
            self.reference_longitude_deg,
            self.reference_altitude_m,
        )

        latitude = np.deg2rad(self.reference_latitude_deg)
        longitude = np.deg2rad(self.reference_longitude_deg)
        sine_latitude = np.sin(latitude)
        cosine_latitude = np.cos(latitude)
        sine_longitude = np.sin(longitude)
        cosine_longitude = np.cos(longitude)
        self._ecef_to_ned = np.array(
            [
                [
                    -sine_latitude * cosine_longitude,
                    -sine_latitude * sine_longitude,
                    cosine_latitude,
                ],
                [-sine_longitude, cosine_longitude, 0.0],
                [
                    -cosine_latitude * cosine_longitude,
                    -cosine_latitude * sine_longitude,
                    -sine_latitude,
                ],
            ]
        )

    def position_ned(
        self, latitude_deg: float, longitude_deg: float, altitude_m: float
    ) -> np.ndarray:
        """Return a geodetic position in reference-centred NED metres."""
        position_ecef = geodetic_to_ecef(
            latitude_deg, longitude_deg, altitude_m
        )
        return self._ecef_to_ned @ (position_ecef - self._reference_ecef)
