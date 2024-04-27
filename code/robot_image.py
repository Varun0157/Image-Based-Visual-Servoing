from PIL import Image, ImageDraw
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


def write_error(img_path: str, error: str) -> None:
    img = Image.open(img_path)
    draw = ImageDraw.Draw(img)

    text = f"error: {error}"
    draw.text((10, 10), text, fill="red")

    img.save(img_path)


def save_rgb_image(img_arr: np.ndarray, index: int) -> str:
    IMG_NAME = f"./rgbimage_{index}.png"
    img = Image.fromarray(img_arr)
    img.save(IMG_NAME)

    return IMG_NAME


def save_with_error(img_arr: np.ndarray, index: int, error: str) -> None:
    img_path = save_rgb_image(img_arr, index)
    write_error(img_path, error)
