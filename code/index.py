import pybullet as p
import pybullet_data

from time import sleep

# import numpy as np

from robot_image import convertRobotImageToArr


def initPyBullet() -> tuple[int, float]:
    pClient = p.connect(p.GUI)
    dt: float = 3 * 1e-4
    p.setAdditionalSearchPath(pybullet_data.getDataPath())
    p.setGravity(0, 0, -10)
    p.setRealTimeSimulation(True)

    return pClient, dt


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
    texture_id = p.loadTexture("qrcode.png")
    p.changeVisualShape(obstacles[0], -1, textureUniqueId=texture_id)

    sleep(1)

    while True:
        pass    


if __name__ == "__main__":
    main()
