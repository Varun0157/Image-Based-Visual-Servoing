"""
responsible for determining robot velocity required based on current image 
"""

import numpy as np
from cv2 import imread
from typing import List

from servo import get_marker_corners

LAMBDA = 1
required_pos = None


def getRequiredPos() -> List[List[float]]:
    """
    lazily loads the required positions of the corners in the aruco marker
    """
    global required_pos

    if required_pos is None:
        frame = imread("static/target.png")
        required_pos = get_marker_corners(frame)
        print(f"required features: {required_pos}")
        assert required_pos is not None, "no Aruco marker found in the goal image"
    return required_pos


def jacobian(X: float, Y: float, Z: float = 1.0) -> List[List[float]]:
    """
    returns the jacobian for the given point and depth
    """
    x = X / Z
    y = Y / Z
    return [
        [-1 / Z, 0, x / Z, x * y, -(1 + x**2), y],
        [0, -1 / Z, Y / Z, (1 + y**2), -x * y, -x],
    ]


def get_velocity(
    points: List[List[float]], depth_buffer: List[List[float]]
) -> np.ndarray:
    """
    gets the velocity vector given the points and the depth buffer
    NOTE: we have 8 features and only 6 dof. In this case we naively take the first three points.
    """

    error = get_error_vec(points)

    J = np.vstack(
        [
            jacobian(
                X=int(points[i][0]),
                Y=int(points[i][1]),
                Z=(depth_buffer[int(points[i][0])][int(points[i][1])]),
            )
            for i in range(3)
        ]
    )
    J_pinv = np.linalg.pinv(J)

    vel = -LAMBDA * np.matmul(J_pinv, error)
    return vel


def get_error_vec(points: List[List[float]]) -> np.ndarray:
    """
    returns an error vector given the observed corner features
    """

    reqPos = getRequiredPos()
    error = np.array([points[i][j] - reqPos[i][j] for i in range(3) for j in range(2)])
    return error


def get_error_mag(error: np.ndarray) -> float:
    """
    returns the magnitude of error for the given error vector
    """
    MSE = np.mean(error**2)
    return float(MSE)
