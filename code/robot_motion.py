import numpy as np
import cv2 as cv
from servo import servo
from typing import List

requiredPos = None

def getRequiredPos() -> List[List[int]]:
    global requiredPos
    if requiredPos is None:
        frame = cv.imread("target.png")
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


def get_error(points: List[List[int]]):
    # randomly acquire a length 3 list of indexes in range(4)
    indices = [0, 1, 2] # consider randomising later
    reqPos = getRequiredPos()
    error = np.array([reqPos[i][j] - points[i][j] for i in indices for j in range(2)])
    return error


def get_velocity(points: List[List[int]]):
    error = get_error(points)

    J = []
    for i in range(3):
        J += jacobian(u=points[i][0], v=points[i][1], z=1)
    J_inv = np.linalg.pinv(J)  # NOTE: implementation takes inv

    vel = np.matmul(J_inv, error)
    # converting this velocity from image frame to robot frame
    # velocity = np.array([vel[2], -vel[1], -vel[0], vel[5], -vel[3], -vel[4]])
    return vel
