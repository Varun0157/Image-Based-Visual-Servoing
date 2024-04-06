import pybullet as p
from PIL import Image

def save_image(pb: p, index: int) -> None:
    img = pb.getCameraImage(224, 224, renderer=pb.ER_BULLET_HARDWARE_OPENGL)
    rgbBuffer = img[2]
    rgbim = Image.fromarray(rgbBuffer)
    rgbim.save(f"rgbimage_{index}.png")


