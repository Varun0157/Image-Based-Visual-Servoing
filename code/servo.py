from functools import cmp_to_key
import numpy as np
import cv2 as cv
from typing import List, Union, Tuple

# NOTE: this implementation of visual servoing uses Aruco markers, which is why we can simply use detectMarkers of the cv.aruco module


def get_markers(img_arr: np.ndarray) -> Tuple[Union[List, None], List]:
    # detect the aruco marker in the image
    marker_dict = cv.aruco.getPredefinedDictionary(cv.aruco.DICT_4X4_1000)
    param_markers = cv.aruco.DetectorParameters()
    detector = cv.aruco.ArucoDetector(marker_dict, param_markers)
    gray_frame = cv.cvtColor(img_arr, cv.COLOR_BGR2GRAY)
    marker_corners, marker_ids, _ = detector.detectMarkers(gray_frame)

    return marker_corners, marker_ids


def mark_center(img_arr: np.ndarray, points: List[List[int]]) -> None:
    center = [
        int(round(sum([point[0] for point in points]) / len(points))),
        int(round(sum([point[1] for point in points]) / len(points))),
    ]

    cv.putText(
        img_arr,
        f"center",
        center,
        cv.FONT_HERSHEY_SIMPLEX,
        1.3,
        (200, 100, 0),
        2,
        cv.LINE_AA,
    )


def servo(img_arr: np.ndarray) -> Union[List[List[float]], None]:
    marker_corners, marker_ids = get_markers(img_arr)
    if not marker_corners:
        return None

    corners, ids = marker_corners[0], marker_ids[0]
    cv.polylines(
        img_arr, [corners.astype(np.int32)], True, (0, 255, 255), 4, cv.LINE_AA
    )

    corners = corners.reshape(4, 2)
    corners = corners.astype(int)

    top_left = list(corners[0].ravel())
    top_right = list(corners[1].ravel())
    bottom_right = list(corners[2].ravel())
    bottom_left = list(corners[3].ravel())

    points = [top_left, top_right, bottom_left, bottom_right]
    points.sort()
    if points[0][1] > points[1][1]:
        points[0][1], points[1][1] = points[1][1], points[0][1]
    if points[2][1] > points[3][1]:
        points[2][1], points[3][1] = points[3][1], points[2][1]

    # mark_center(img_arr, points)

    return points
