from PIL import Image 
import numpy as np

def convertRobotImageToArr(arr: list[list[list[int]]], h: int, w: int) -> np.ndarray:
    pixels = []
    for x in range(h):
        for y in range(w):
            r, g, b, a = arr[x][y]
            pixels.append((r, g, b, a))
    
    img = Image.new('RGBA', (w, h))
    img.putdata(pixels)
    return np.array(img)
