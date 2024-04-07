import pybullet as p
import pybullet_data

import time 

from save_image import save_image

def main() -> None:
    physicsClient = p.connect(p.DIRECT)#or p.DIRECT for non-graphical version
    # NOTE: this sim code is completely as in the pybullet docs. 
    p.setAdditionalSearchPath(pybullet_data.getDataPath()) #optionally
    p.setGravity(0,0,-10)
    planeId = p.loadURDF("plane.urdf")
    startPos = [0,0,1]
    startOrientation = p.getQuaternionFromEuler([0,0,0])
    boxId = p.loadURDF("r2d2.urdf",startPos, startOrientation)
    #set the center of mass frame (loadURDF sets base link frame) startPos/Ornp.resetBasePositionAndOrientation(boxId, startPos, startOrientation)
    for i in range (250):
        p.stepSimulation()
        time.sleep(1./240.)
        save_image(p, i)
    
    cubePos, cubeOrn = p.getBasePositionAndOrientation(boxId)
    print(cubePos,cubeOrn)

    p.disconnect()

if __name__ == "__main__":
    main()

