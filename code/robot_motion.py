import numpy as np
import cv2 as cv
from servo import servo
from typing import List

LAMBDA = 1
requiredPos = None


def getRequiredPos() -> List[List[float]]:
    global requiredPos

    if requiredPos is None:
        frame = cv.imread("target.png")
        requiredPos, _ = servo(frame)
        print(requiredPos)
        assert requiredPos is not None, "no Aruco marker found in the goal image"
    return requiredPos


def jacobian(X: float, Y: float, Z: float = 1.0) -> List[List[float]]:
    # NOTE: confirm this with the paper jacobian later. I don't think this is fully right.
    x = X / Z
    y = Y / Z
    return [
        [-1 / Z, 0, x / Z, x * y, -(1 + x**2), y],
        [0, -1 / Z, Y / Z, (1 + y**2), -x * y, -x],
    ]


def get_velocity(
    points: List[List[float]], depth_buffer: List[List[float]]
) -> np.ndarray:
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
    # create a get_score that takes a points list, final error is difference between both in 6 dof

    # consider randomly acquire a length 3 list of indexes in range(4)
    indices = [0, 1, 2]
    reqPos = getRequiredPos()
    error = np.array(
        [
            points[indices[i]][j] - reqPos[indices[i]][j]
            for i in range(3)
            for j in range(2)
        ]
    )
    return error


def get_error_mag(error: np.ndarray) -> float:
    MSE = np.mean(error**2)
    return float(MSE)


MIN_ERROR = float("inf")
