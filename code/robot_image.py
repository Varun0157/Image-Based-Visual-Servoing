from PIL import Image, ImageDraw
from matplotlib.pylab import f
import numpy as np
from typing import List, Tuple


def convertRobotImageToArr(arr: List, h: int, w: int) -> np.ndarray:
    pixels = []
    for x in range(h):
        for y in range(w):
            r, g, b, a = arr[x][y]
            pixels.append((r, g, b, a))

    img = Image.new("RGBA", (w, h))
    img.putdata(pixels)
    return np.array(img)


def write_text_to_image(
    img_path: str, text: str, xy: Tuple[int, int], color: str
) -> None:
    img = Image.open(img_path)

    draw = ImageDraw.Draw(img)
    draw.text(xy, text, fill=color)

    img.save(img_path)


def save_rgb_image(img_arr: np.ndarray, img_path: str) -> None:
    img = Image.fromarray(img_arr)
    img.save(img_path)


def save_with_error(
    img_path: str, img_arr: np.ndarray, index: int, error: float
) -> None:
    save_rgb_image(img_arr, img_path)
    write_text_to_image(img_path, f"error: {error}", (10, 10), "red")
