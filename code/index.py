"""
main IBVS module 
"""

import pybullet as p, pybullet_data

import sys
from time import sleep
from typing import Tuple, List

import numpy as np

from robot_image import convert_img_to_arr, save_image, get_image_config
from servo import get_marker_corners, mark_corners
from robot_motion import get_error_mag, get_error_vec, get_velocity

MAX_ITERATIONS = int(1e3)


def initPyBullet() -> int:
    """
    initialises the pybullet scene
    """
    pClient = p.connect(p.DIRECT)
    p.setAdditionalSearchPath(pybullet_data.getDataPath())
    p.setGravity(0, 0, -10)
    # p.setRealTimeSimulation(True)

    return pClient


def set_aruco_marker_texture(obstacle_id: int) -> None:
    """
    sets the aruco marker texture on the obstacle
    """
    texture_id = p.loadTexture("aruco_marker.png")
    p.changeVisualShape(obstacle_id, -1, textureUniqueId=texture_id)


def init_scene(robot_pos: list[float]) -> Tuple[int, List[int]]:
    """
    initialises the scene and returns created objects
    """

    plane_id = p.loadURDF("plane.urdf")
    # loading obstacles, with the main cube at the first index
    base = robot_pos  # for now
    base_orn = p.getQuaternionFromEuler([0, 0, 0])
    obstacles = []
    for z_offset in [0, 1]:
        for y_offset in [7.5]:
            for x_offset in [0, -1, 1]:
                obstacles.append(
                    p.loadURDF(
                        "cube_small.urdf",
                        [
                            base[0] + x_offset,
                            base[1] + y_offset,
                            base[2] + z_offset,
                        ],
                        base_orn,
                        globalScaling=20,
                    )
                )

    goal_obs_id = obstacles[0]  # taking the 0th as the goal
    set_aruco_marker_texture(goal_obs_id)

    return plane_id, obstacles


def get_robot_rotation_matrix(robot_orientation: List[float]) -> np.ndarray:
    """
    gets the robot rotation matrix on the basis of the orientation
    """
    robot_rotation_matrix = p.getMatrixFromQuaternion(
        p.getQuaternionFromEuler(robot_orientation)
    )
    return np.array(object=robot_rotation_matrix).reshape(3, 3)


def get_view_matrix(
    init_camera_vector: Tuple[int, int, int],
    init_up_vector: Tuple[int, int, int],
    robot_pos: List[float],
    robot_rotation_matrix: np.ndarray,
) -> np.ndarray:
    """
    get the view matrix for the camera
    """
    camera_vector = np.dot(robot_rotation_matrix, init_camera_vector)
    up_vector = np.dot(robot_rotation_matrix, init_up_vector)
    return p.computeViewMatrix(robot_pos, robot_pos + camera_vector, up_vector)


def get_projection_matrix() -> np.ndarray:
    """
    get the projection matrix for the camera
    """
    image_conf = get_image_config()
    return p.computeProjectionMatrixFOV(
        image_conf["fov"],
        image_conf["aspect"],
        image_conf["near_val"],
        image_conf["far_val"],
    )


def get_transformation_matrix(
    robot_pos: List[float], robot_rotation_matrix: np.ndarray
) -> np.ndarray:
    """
    create the transformation matrix to convert from image (world) coordinates to local camera coordinates
    """
    # creating the transformation matrix

    # translate the camera to the robot position
    T_c = np.zeros((4, 4))
    for i, val in enumerate([-robot_pos[0], -robot_pos[1], -robot_pos[2], 1]):
        T_c[i][3] = val
        T_c[i][i] = 1

    # rotate the camera to the robot orientation
    R_i = np.zeros((4, 4))
    for i in range(3):
        for j in range(3):
            R_i[i][j] = robot_rotation_matrix[i][j]
    R_i[3][3] = 1

    transform = np.matmul(R_i, T_c)

    return transform


def capture_camera_image(
    robot_pos: List[float], robot_rotation_matrix: np.ndarray
) -> np.ndarray:
    """
    captures the image from the camera on the basis of the robot position and rotation matrix
    """
    image_conf = get_image_config()

    # robot rotation matrix
    # initial camera vectors
    init_camera_vector = (0, 1, 0)  # y axis
    init_up_vector = (0, 0, 1)  # z axis

    # calculating the view matrix
    view_matrix = get_view_matrix(
        init_camera_vector,
        init_up_vector,
        robot_pos,
        robot_rotation_matrix,
    )
    # calculating the projection matrix
    projection_matrix = get_projection_matrix()

    # capturing the image
    img_details = p.getCameraImage(
        image_conf["width"],
        image_conf["height"],
        view_matrix,
        projection_matrix,
    )
    return img_details


def update_pos_and_orn(
    transform: np.ndarray,
    velocity: np.ndarray,
    robot_pos: List[float],
    robot_orn: List[float],
    dt: float,
) -> Tuple[List[float], List[float]]:
    """
    returns the updated position and orientation of the robot
    uses homogenous coordinates
    """
    del_pos = np.matmul(transform, [*velocity[:3], 1])
    for i in range(3):
        robot_pos[i] += (del_pos[i] / del_pos[-1]) * dt

    del_orn = np.matmul(transform, [*velocity[3:], 1])
    for i in range(3):
        robot_orn[i] += (del_orn[i] / del_orn[-1]) * dt

    return robot_pos, robot_orn


MIN_ERROR = float("inf")
ERROR_GROWTH_LIMIT = 0.05  # 5%


def update_error(error_mag: float, i: int | None = None) -> None:
    """
    updates the global min error and determines when to break
    """

    global MIN_ERROR

    if i is not None:
        print(f"{i}:", end="")
    if error_mag < MIN_ERROR:
        MIN_ERROR = error_mag
        print(f"new min error: {MIN_ERROR}")
    else:
        if error_mag > MIN_ERROR:
            print(f"increase from min: {error_mag - MIN_ERROR}")
        else:
            print(f"error remained same: {error_mag}")

    if error_mag > (1 + ERROR_GROWTH_LIMIT) * MIN_ERROR:
        # indicator of some divergence
        print("DONE")
        p.disconnect()
        sys.exit(0)


def main() -> None:
    """
    the main flow
    """
    _ = initPyBullet()
    img_conf = get_image_config()
    dt: float = 0.0001

    # initialise the robot
    robot_pos = [0, 0, 1.0]
    robot_orientation = [0, 0, 0 - np.pi / 3]

    plane_id, obstacles = init_scene(robot_pos)

    sleep(5)  # arbitrary sleep to let the scene load
    for i in range(MAX_ITERATIONS):
        p.stepSimulation()

        robot_rot_matrix = get_robot_rotation_matrix(robot_orientation)

        img = capture_camera_image(robot_pos, robot_rot_matrix)

        rgb_img_arr = convert_img_to_arr(
            img[2], int(img_conf["height"]), int(img_conf["width"])
        )

        servo_points = get_marker_corners(rgb_img_arr)
        # img_arr = mark_corners(img_arr, points) # uncomment this line to mark the corners of the aruco marker

        if not servo_points:
            error = None
            save_image(error, i, rgb_img_arr, MIN_ERROR)
            print("no aruco marker detected, rotating")
            robot_orientation[2] += np.pi / 18
            continue

        error = get_error_mag(get_error_vec(servo_points))
        save_image(error, i, rgb_img_arr, MIN_ERROR)
        update_error(error, i=i)

        velocity = get_velocity(points=servo_points, depth_buffer=img[3])
        transform = get_transformation_matrix(robot_pos, robot_rot_matrix)

        robot_pos, robot_orientation = update_pos_and_orn(
            transform, velocity, robot_pos, robot_orientation, dt
        )

        sleep(0.01)  # arbitrary sleep to let the changes take place


if __name__ == "__main__":
    main()
