import sys, os
import pybullet as p
import pybullet_data

from time import sleep
from typing import Tuple, Union, Dict, List

import numpy as np

from robot_image import convertRobotImageToArr, save_rgb_image, save_with_error
from servo import servo
from robot_motion import get_error_mag, get_error_vec, get_velocity

MAX_ITERATIONS = int(1e3)


def initPyBullet() -> int:
    pClient = p.connect(p.DIRECT)
    p.setAdditionalSearchPath(pybullet_data.getDataPath())
    p.setGravity(0, 0, -10)
    # p.setRealTimeSimulation(True)

    return pClient


def get_image_config() -> Dict[str, Union[int, float]]:
    WIDTH = 800
    HEIGHT = 500
    FOV = 90
    NEAR_VAL = 0.01
    FAR_VAL = 100

    # make this functional, returning a bunch of gets
    # also consider moving it to robot_image
    return {
        "width": WIDTH,
        "height": HEIGHT,
        "fov": FOV,
        "near_val": NEAR_VAL,
        "far_val": FAR_VAL,
        "aspect": WIDTH / HEIGHT,
    }


def init_scene(robot_pos: list[float]) -> Tuple[int, List[int]]:
    plane_id = p.loadURDF("plane.urdf")
    # loading obstacles, with the main cube at the first index
    base = robot_pos  # for now
    base_orn = p.getQuaternionFromEuler([0, 0, 0])
    obstacles = []
    for z_offset in [0, 1]:
        for y_offset in [5]:
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

    sleep(2)

    return plane_id, obstacles


def set_aruco_marker_texture(obstacle_id: int) -> None:
    texture_id = p.loadTexture("aruco_marker.png")
    p.changeVisualShape(obstacle_id, -1, textureUniqueId=texture_id)


def get_robot_rotation_matrix(robot_orientation: List[float]) -> np.ndarray:
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
    camera_vector = robot_rotation_matrix.dot(init_camera_vector)
    up_vector = robot_rotation_matrix.dot(init_up_vector)
    return p.computeViewMatrix(robot_pos, robot_pos + camera_vector, up_vector)


def get_projection_matrix() -> np.ndarray:
    image_conf = get_image_config()
    return p.computeProjectionMatrixFOV(
        image_conf["fov"],
        image_conf["aspect"],
        image_conf["near_val"],
        image_conf["far_val"],
    )


def get_transformation_matrix(
    robot_pos: List[float], robot_orientation: List[float]
) -> np.ndarray:
    # creating the transformation matrix
    robot_rotation_matrix = get_robot_rotation_matrix(robot_orientation)
    R_i = np.zeros((4, 4))
    for i in range(3):
        for j in range(3):
            R_i[i][j] = robot_rotation_matrix[i][j]
    R_i[3][3] = 1

    T_c = np.zeros((4, 4))
    for i, val in enumerate([-robot_pos[0], -robot_pos[1], -robot_pos[2], 1]):
        T_c[i][3] = val
        T_c[i][i] = 1
    transform = np.matmul(R_i, T_c)

    # print(transform)
    return transform


def capture_camera_image(
    robot_pos: List[float],
    robot_orientation: List[float],
) -> np.ndarray:
    # robot rotation matrix
    # initial camera vectors
    init_camera_vector = (0, 1, 0)  # y axis
    init_up_vector = (0, 0, 1)  # z axis

    # calculating the view matrix
    view_matrix = get_view_matrix(
        init_camera_vector,
        init_up_vector,
        robot_pos,
        get_robot_rotation_matrix(robot_orientation),
    )
    # calculating the projection matrix
    projection_matrix = get_projection_matrix()

    # capturing the image
    image_conf = get_image_config()
    img_details = p.getCameraImage(
        image_conf["width"],
        image_conf["height"],
        view_matrix,
        projection_matrix,
    )
    return img_details


MIN_ERROR = float("inf")
ERROR_GROWTH_LIMIT = 1.2


def update_error(servo_points: List[List[float]], i: int | None = None) -> None:
    global MIN_ERROR

    error = get_error_mag(get_error_vec(servo_points))

    if i is not None:
        print(f"{i}:", end="")
    if error < MIN_ERROR:
        MIN_ERROR = error
        print(f"new min error: {MIN_ERROR}")
    else:
        print(f"increase in error: {error - MIN_ERROR}")

    if error > ERROR_GROWTH_LIMIT * MIN_ERROR:
        # sign of divergence
        print("DONE")
        p.disconnect()
        sys.exit(0)


def save_image(
    servo_points: List[List[float]] | None, i: int, img_arr: np.ndarray
) -> None:
    # create the img directory if it does not exist
    if not os.path.exists("img"):
        os.makedirs("img")

    global MIN_ERROR

    error_mag = get_error_mag(get_error_vec(servo_points)) if servo_points else None
    error_str = f"{error_mag:.2f}" if error_mag else "undefined"

    save_with_error(
        img_arr,
        f"./img/rgbimage_{i}.png",
        error_str,
        "green" if error_mag and error_mag < MIN_ERROR else "red",
    )


def update_pos_and_orn(
    transform: np.ndarray,
    velocity: np.ndarray,
    robot_pos: List[float],
    robot_orn: List[float],
    dt: float,
) -> Tuple[List[float], List[float]]:
    del_pos = np.matmul(transform, [*velocity[:3], 1])
    for i in range(3):
        robot_pos[i] += (del_pos[i] / del_pos[-1]) * dt

    del_orn = np.matmul(transform, [*velocity[3:], 1])
    for i in range(3):
        robot_orn[i] += (del_orn[i] / del_orn[-1]) * dt

    return robot_pos, robot_orn


def main() -> None:
    _ = initPyBullet()
    dt: float = 0.0001

    # initialise the robot
    robot_pos = [0, 0, 1.0]
    robot_orientation = [0, 0, 0 - np.pi / 3]

    plane_id, obstacles = init_scene(robot_pos)

    sleep(5)
    for i in range(MAX_ITERATIONS):
        p.stepSimulation()

        rgb_img = capture_camera_image(robot_pos, robot_orientation)[2]

        img_conf = get_image_config()
        img_arr = convertRobotImageToArr(
            rgb_img, int(img_conf["height"]), int(img_conf["width"])
        )

        servo_points = servo(img_arr)
        save_image(servo_points, i, img_arr)

        if not servo_points:
            print("no aruco marker detected, rotating")
            robot_orientation[2] += np.pi / 18
            continue

        update_error(servo_points=servo_points, i=i)

        velocity = get_velocity(points=servo_points)
        transform = get_transformation_matrix(robot_pos, robot_orientation)

        robot_pos, robot_orientation = update_pos_and_orn(
            transform, velocity, robot_pos, robot_orientation, dt
        )

        sleep(0.01)


if __name__ == "__main__":
    main()
