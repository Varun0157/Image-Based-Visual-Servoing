import pybullet as p
from PIL import Image

RES = (2048, 2048)

def save_image(pb: p, index: int) -> None:
    img = pb.getCameraImage(*RES, renderer=pb.ER_BULLET_HARDWARE_OPENGL)
    rgbBuffer = img[2]
    rgbim = Image.fromarray(rgbBuffer)
    rgbim.save(f"./img/rgbimage_{index}.png")


