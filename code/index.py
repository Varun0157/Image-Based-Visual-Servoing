import pybullet as p
import pybullet_data

from time import sleep
from typing import Union, Dict

import numpy as np

from robot_image import convertRobotImageToArr, save_rgb_image
from servo import servo
from robot_motion import get_error, get_velocity

MAX_ITERATIONS = int(1e4)


def initPyBullet() -> int:
    pClient = p.connect(p.DIRECT)
    p.setAdditionalSearchPath(pybullet_data.getDataPath())
    p.setGravity(0, 0, -10)
    # p.setRealTimeSimulation(1)

    return pClient


def get_image_config() -> Dict[str, Union[int, float]]:
    WIDTH = 800
    HEIGHT = 600
    FOV = 90
    NEAR_VAL = 0.01
    FAR_VAL = 100

    return {
        "width": WIDTH,
        "height": HEIGHT,
        "fov": FOV,
        "near_val": NEAR_VAL,
        "far_val": FAR_VAL,
        "aspect": WIDTH / HEIGHT,
    }


def main() -> None:
    _ = initPyBullet()
    dt: float = 0.0001

    # initialise the plane
    plane_id = p.loadURDF("plane.urdf")

    # initialise the robot
    robot_start_pos = [0, 0, 1]
    robot_start_orientation = p.getQuaternionFromEuler([0, 0, 0 - (np.pi / 4)])
    robot_id = p.loadURDF("r2d2.urdf", robot_start_pos, robot_start_orientation)

    # loading obstacles, with the main cube at the first index
    base = robot_start_pos  # for now
    base_orn = p.getQuaternionFromEuler([0, 0, 0])
    obstacles = []
    for x_offset in [0, -1, 1]:
        for y_offset in [5]:
            for z_offset in [0, 1]:
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

    # taking [0, 5, 0] as the obstacle with the qrcode
    goal_obs_id = obstacles[0]
    texture_id = p.loadTexture("aruco_marker.png")
    p.changeVisualShape(goal_obs_id, -1, textureUniqueId=texture_id)

    sleep(1)

    MAX_ERROR = 1
    for i in range(MAX_ITERATIONS):
        # getting the robot and the obstacle with the qrcode
        robot_pos, robot_orientation = p.getBasePositionAndOrientation(robot_id)
        goal_pos, goal_orientation = p.getBasePositionAndOrientation(goal_obs_id)

        # getting the rotation matrix
        robot_rotation_matrix = p.getMatrixFromQuaternion(robot_orientation)
        robot_rotation_matrix = np.array(object=robot_rotation_matrix).reshape(3, 3)

        # initial camera vectors
        init_camera_vector = (0, 1, 0)  # y axis
        init_up_vector = (0, 0, 1)  # z axis

        # rotated vectors
        approx_view_z_offset = 0.7
        robot_view_point = [
            robot_pos[0],
            robot_pos[1],
            robot_pos[2] + approx_view_z_offset,
        ]
        camera_vector = robot_rotation_matrix.dot(init_camera_vector)
        up_vector = robot_rotation_matrix.dot(init_up_vector)
        view_matrix = p.computeViewMatrix(
            robot_view_point, robot_pos + 2 * camera_vector, up_vector
        )

        p.stepSimulation()

        image_conf = get_image_config()
        projection_matrix = p.computeProjectionMatrixFOV(
            image_conf["fov"],
            image_conf["aspect"],
            image_conf["near_val"],
            image_conf["far_val"],
        )
        img_details = p.getCameraImage(
            image_conf["width"],
            image_conf["height"],
            view_matrix,
            projection_matrix,
        )

        rgb_img = img_details[2]
        img_arr = convertRobotImageToArr(
            rgb_img, int(image_conf["height"]), int(image_conf["width"])
        )

        save_rgb_image(img_arr, i)

        servo_points = servo(img_arr)

        if not servo_points:
            print("no aruco marker detected, rotating")
            _, orientation = p.getBasePositionAndOrientation(robot_id)
            euler_orientation = p.getEulerFromQuaternion(orientation)
            p.resetBasePositionAndOrientation(
                robot_id,
                robot_pos,
                p.getQuaternionFromEuler(
                    [
                        euler_orientation[0],
                        euler_orientation[1],
                        euler_orientation[2] + np.pi / 18,
                    ]
                ),
            )
            continue

        # an aruco marker was detected
        error = get_error(servo_points)
        MSE = np.mean(error**2)
        print(f"error: {error}, mse = {MSE}")
        if MSE < 500:
            print("DONE")
            p.disconnect()
            break
        elif MSE > MAX_ERROR:
            MAX_ERROR = MSE

        velocity = get_velocity(servo_points)

        robot_pos = list(robot_pos)
        robot_orientation = list(p.getEulerFromQuaternion(robot_orientation))

        # creating the transformation matrix
        #       [R R R -Tx]
        #       [R R R -Ty]
        #       [R R R -Tz]
        #       [0 0 0  1 ]
        transform = np.zeros((4, 4))
        for i in range(3):
            for j in range(3):
                transform[i][j] = robot_rotation_matrix[i][j]
        for i, val in enumerate([-robot_pos[0], -robot_pos[1], -robot_pos[2], 1]):
            transform[i][3] = val
        # print(transform)

        # NOTE: implementation says velocity[3] = 1 here, but I don't see why, so not adding it
        del_pos = np.matmul(transform, [*velocity[:3], 1])
        # print(f"velocity: {velocity}, delpos:{del_pos}", sep="\n")
        for i in range(3):
            robot_pos[i] += del_pos[i] * dt

        # del_orn = np.matmul(transform, [*velocity[3:], 1])
        # robot_orientation[0] += del_orn[0] * dt * 75
        # robot_orientation[1] += del_orn[1] * dt * 75
        robot_orientation[2] += velocity[5] * dt * 75
        robot_orientation = p.getQuaternionFromEuler(robot_orientation)
        p.resetBasePositionAndOrientation(robot_id, robot_pos, robot_orientation)
        sleep(0.01)


if __name__ == "__main__":
    main()
