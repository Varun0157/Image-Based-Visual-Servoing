from PIL import Image
import numpy as np
from typing import List


def convertRobotImageToArr(arr: List, h: int, w: int) -> np.ndarray:
    pixels = []
    for x in range(h):
        for y in range(w):
            r, g, b, a = arr[x][y]
            pixels.append((r, g, b, a))

    img = Image.new("RGBA", (w, h))
    img.putdata(pixels)
    return np.array(img)


def save_rgb_image(img: List, index: int) -> None:
    rgbBuffer = img[2]
    rgbim = Image.fromarray(rgbBuffer)
    rgbim.save(f"./img/rgbimage_{index}.png")
