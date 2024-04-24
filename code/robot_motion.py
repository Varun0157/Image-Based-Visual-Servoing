import numpy as np
import cv2 as cv
from servo import servo
from typing import List

requiredPos = None


def getRequiredPos() -> List[List[int]]:
    global requiredPos
    if requiredPos is None:
        frame = cv.imread("output_reference.png")
        requiredPos = servo(frame)
        print(requiredPos)

    assert requiredPos is not None, "no Aruco marker found in the goal image"
    return requiredPos


def jacobian(u: float, v: float, f=1, z=1) -> List[List[float]]:
    # NOTE: confirm this with the paper jacobian later. I don't think this is fully right.
    return [
        [-f / z, 0, u / z, u * v / z, -(f + u * u) / f, v],
        [0, -f / z, v / z, (f + v * v) / f, -u * v / f, -u],
    ]


def get_velocity(points: List[List[int]]):
    # NOTE: in implementation, they say in range(3) twice. I think that's wrong and I'm using in range(4)
    reqPos = getRequiredPos()
    error = np.array([reqPos[i][j] - points[i][j] for i in range(4) for j in range(2)])
    print(f"error: {error}")

    J = []
    for i in range(4):
        J += jacobian(u=points[i][0], v=points[i][1], z=1)
    J_inv = np.linalg.pinv(J)  # NOTE: implementation takes inv

    velocity = np.matmul(J_inv, error)
    return velocity
