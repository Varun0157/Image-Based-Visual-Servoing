from PIL import Image, ImageDraw
import numpy as np
from typing import List, Tuple, Dict, Union


def get_image_config() -> Dict[str, Union[int, float]]:
    """
    returns the image configuration
    """

    WIDTH = 800
    HEIGHT = 500
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


def convertRobotImageToArr(arr: List, h: int, w: int) -> np.ndarray:
    """
    converts the robot image into a numpy array
    """
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
    """
    writes any given text to an image
    """
    img = Image.open(img_path)

    draw = ImageDraw.Draw(img)
    draw.text(xy, text, fill=color)

    img.save(img_path)


def save_rgb_image(img_arr: np.ndarray, img_path: str) -> None:
    """
    saves an rgb image to a given path
    """
    img = Image.fromarray(img_arr)
    img.save(img_path)


def save_with_error(
    img_arr: np.ndarray, img_path: str, error: str, color: str = "black"
) -> None:
    """
    saves the image with the error text
    """
    save_rgb_image(img_arr, img_path)
    write_text_to_image(img_path, f"error: {error}", (10, 10), color)
