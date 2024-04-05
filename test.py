import pybullet as p
import time
import pybullet_data
import numpy as np
from PIL import Image 

physicsClient = p.connect(p.DIRECT)#or p.DIRECT for non-graphical version
p.setAdditionalSearchPath(pybullet_data.getDataPath()) #optionally
p.setGravity(0,0,-10)
planeId = p.loadURDF("plane.urdf")
startPos = [0,0,1]
startOrientation = p.getQuaternionFromEuler([0,0,0])
boxId = p.loadURDF("r2d2.urdf",startPos, startOrientation)
#set the center of mass frame (loadURDF sets base link frame) startPos/Ornp.resetBasePositionAndOrientation(boxId, startPos, startOrientation)
for i in range (10000):
    p.stepSimulation()
    time.sleep(1./240.)
cubePos, cubeOrn = p.getBasePositionAndOrientation(boxId)
print(cubePos,cubeOrn)


# attempting to save image 
img = p.getCameraImage(224, 224, renderer=p.ER_BULLET_HARDWARE_OPENGL)
rgbBuffer = img[2] 
rgbim = Image.fromarray(rgbBuffer)
rgbim.save('rgbtest.png')

p.disconnect()
