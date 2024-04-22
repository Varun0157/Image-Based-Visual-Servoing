import pybullet as p
import pybullet_data

from time import sleep

import numpy as np

from robot_image import convertRobotImageToArr


def initPyBullet() -> tuple[int, float]:
    pClient = p.connect(p.GUI)
    dt: float = 3 * 1e-4
    p.setAdditionalSearchPath(pybullet_data.getDataPath())
    p.setGravity(0, 0, -10)
    p.setRealTimeSimulation(True)

    return pClient, dt


def get_image_config() -> dict[str, int | float]:
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
    physicsClient, dt = initPyBullet()

    # initialise the plane
    plane_id = p.loadURDF("plane.urdf")

    # initialise the robot
    robot_start_pos = [0, 0, 1]
    robot_start_orientation = p.getQuaternionFromEuler(
        [0, 0, 0]
    )  # NOTE: in the example it is different
    robot_id = p.loadURDF("robot.urdf", robot_start_pos, robot_start_orientation)

    # loading obstacles, with the main cube at the first index
    base = robot_start_pos  # for now
    obstacles = []
    for x_offset in [0, -1, 1]:
        for y_offset in [5]:
            for z_offset in [0, 5]:
                obstacles.append(
                    p.loadURDF(
                        "cube_small.urdf",
                        [
                            base[0] + x_offset,
                            base[1] + y_offset,
                            base[2] + z_offset,
                        ],
                    )
                )

    # taking [0, 5, 0] as the obstacle with the qrcode
    goal_obs_id = obstacles[0]
    texture_id = p.loadTexture("qrcode.png")
    p.changeVisualShape(goal_obs_id, -1, textureUniqueId=texture_id)

    sleep(1)

    while True:
        # getting the robot and the obstacle with the qrcode
        robot_pos, robot_orientation = p.getBasePositionAndOrientation(robot_id)
        goal_pos, goal_orientation = p.getBasePositionAndOrientation(goal_obs_id)

        # getting the rotation matrix
        robot_rotation_matrix = p.getMatrixFromQuaternion(robot_orientation)
        robot_rotation_matrix = np.array(object=robot_rotation_matrix).reshape(3, 3)

        # initial camera vectors
        init_camera_vector = [0, 1, 0]  # y axis
        init_up_vector = [0, 0, 1]  # z axis

        # rotated vectors
        approx_view_z_offset = 0.6
        robot_view_point = [
            robot_pos[0],
            robot_pos[1],
            robot_pos[2] + approx_view_z_offset,
        ]
        camera_vector = robot_rotation_matrix.dot(init_camera_vector)
        up_vector = robot_rotation_matrix.dot(init_up_vector)
        view_matrix = p.computeViewMatrix(
            robot_view_point, robot_pos + camera_vector, up_vector
        )

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

        img_arr = convertRobotImageToArr(
            img_details, int(image_conf["height"]), int(image_conf["width"])
        )

        # servo_points = servoing(arr)
        


        pass


if __name__ == "__main__":
    main()
