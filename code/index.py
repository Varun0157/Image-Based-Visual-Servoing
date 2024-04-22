import pybullet as p
import pybullet_data

from time import sleep 

import numpy as np 

from PIL import Image 

def convertRobotImageToArr(arr: list[list[list[int]]], h: int, w: int) -> np.ndarray[np.Any, np.dtype[np.Any]]:
    pixels = []
    for x in range(h):
        for y in range(w):
            r, g, b, a = arr[x][y]
            pixels.append((r, g, b, a))
    
    img = Image.new('RGBA', (w, h))
    img.putdata(pixels)

    return np.array(img)


if __name__ == "__main__":
    main()

