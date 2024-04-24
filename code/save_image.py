from PIL import Image
import numpy as np

RES = (2048, 2048)


def save_rgb_image(img, index: int) -> None:
    rgbBuffer = img[2]
    rgbim = Image.fromarray(rgbBuffer)
    rgbim.save(f"./img/rgbimage_{index}.png")
