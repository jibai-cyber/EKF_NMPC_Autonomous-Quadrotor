"""Explicit conversion between Gazebo ENU/FLU and project NED/FRD."""

import numpy as np

from .dynamics import normalize_quaternion, quaternion_to_rotation


ENU_TO_NED = np.array([[0.0, 1.0, 0.0], [1.0, 0.0, 0.0], [0.0, 0.0, -1.0]])
FRD_TO_FLU = np.diag([1.0, -1.0, -1.0])


def vector_enu_to_ned(vector: np.ndarray) -> np.ndarray:
    return ENU_TO_NED @ np.asarray(vector, dtype=float)


def vector_flu_to_frd(vector: np.ndarray) -> np.ndarray:
    # This transform is self-inverse.
    return FRD_TO_FLU @ np.asarray(vector, dtype=float)


def rotation_to_quaternion(rotation: np.ndarray) -> np.ndarray:
    """Convert a proper 3x3 rotation matrix to Hamilton [w, x, y, z]."""
    rotation = np.asarray(rotation, dtype=float)
    trace = np.trace(rotation)
    if trace > 0.0:
        scale = 2.0 * np.sqrt(trace + 1.0)
        quaternion = np.array(
            [
                0.25 * scale,
                (rotation[2, 1] - rotation[1, 2]) / scale,
                (rotation[0, 2] - rotation[2, 0]) / scale,
                (rotation[1, 0] - rotation[0, 1]) / scale,
            ]
        )
    else:
        diagonal = np.diag(rotation)
        index = int(np.argmax(diagonal))
        if index == 0:
            scale = 2.0 * np.sqrt(1.0 + rotation[0, 0] - rotation[1, 1] - rotation[2, 2])
            quaternion = np.array(
                [
                    (rotation[2, 1] - rotation[1, 2]) / scale,
                    0.25 * scale,
                    (rotation[0, 1] + rotation[1, 0]) / scale,
                    (rotation[0, 2] + rotation[2, 0]) / scale,
                ]
            )
        elif index == 1:
            scale = 2.0 * np.sqrt(1.0 - rotation[0, 0] + rotation[1, 1] - rotation[2, 2])
            quaternion = np.array(
                [
                    (rotation[0, 2] - rotation[2, 0]) / scale,
                    (rotation[0, 1] + rotation[1, 0]) / scale,
                    0.25 * scale,
                    (rotation[1, 2] + rotation[2, 1]) / scale,
                ]
            )
        else:
            scale = 2.0 * np.sqrt(1.0 - rotation[0, 0] - rotation[1, 1] + rotation[2, 2])
            quaternion = np.array(
                [
                    (rotation[1, 0] - rotation[0, 1]) / scale,
                    (rotation[0, 2] + rotation[2, 0]) / scale,
                    (rotation[1, 2] + rotation[2, 1]) / scale,
                    0.25 * scale,
                ]
            )
    return normalize_quaternion(quaternion)


def quaternion_enu_flu_to_ned_frd(quaternion_enu_flu: np.ndarray) -> np.ndarray:
    rotation_enu_flu = quaternion_to_rotation(quaternion_enu_flu)
    rotation_ned_frd = ENU_TO_NED @ rotation_enu_flu @ FRD_TO_FLU
    return rotation_to_quaternion(rotation_ned_frd)

