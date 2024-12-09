#!/usr/bin/env python3
from PIL import Image, ImageOps
import numpy as np
import matplotlib.pyplot as plt

def show():
    grid = np.load('cspace.npy')
    plt.imshow(grid, cmap = "binary")
    plt.tight_layout()
    plt.show()


def image():
    img = Image.open('map.png')
    img = ImageOps.grayscale(img)
    np_img = np.array(img)
    np_img = ~np_img # invert Black & White
    # np_img[np_img > 0] = 1
    print(np_img)

    plt.set_cmap('binary')
    plt.imshow(np_img)

    np.save('cspace.npy', np_img)

image()    
show()