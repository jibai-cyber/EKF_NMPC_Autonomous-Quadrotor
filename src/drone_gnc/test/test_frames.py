import numpy as np

from drone_gnc.dynamics import quaternion_to_rotation
from drone_gnc.frames import (
    ENU_TO_NED,
    FRD_TO_FLU,
    quaternion_enu_flu_to_ned_frd,
    vector_enu_to_ned,
    vector_flu_to_frd,
)


def test_vector_conversions():
    np.testing.assert_allclose(vector_enu_to_ned([1.0, 2.0, 3.0]), [2.0, 1.0, -3.0])
    np.testing.assert_allclose(vector_flu_to_frd([1.0, 2.0, 3.0]), [1.0, -2.0, -3.0])


def test_attitude_conversion_is_consistent():
    q_enu_flu = np.array([1.0, 0.0, 0.0, 0.0])
    q_ned_frd = quaternion_enu_flu_to_ned_frd(q_enu_flu)
    np.testing.assert_allclose(
        quaternion_to_rotation(q_ned_frd), ENU_TO_NED @ FRD_TO_FLU, atol=1e-12
    )

