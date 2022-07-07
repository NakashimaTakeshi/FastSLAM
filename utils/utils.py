import os

import matplotlib.pyplot as plt
from PIL import Image


def rescale_list(list_, min_value, max_value):
    """
    Args:
        list_: list to be rescaled.
        min_value: minimum value of rescaled list.
        max_value: maximum value of rescaled list.
    Returns:
        rescaled list.
    """
    return [(x - min(list_)) / (max(list_) - min(list_)) * (max_value - min_value) + min_value for x in list_]


def save_png():
    directorypath = "./data/output/"
    counter = png_file_counter(directorypath)
    plt.savefig(directorypath + str(counter) + '.png', bbox_inches='tight')


def png_file_counter(directorypath):
    counter: int = 0
    while os.path.exists(directorypath + str(counter) + ".png"):
        counter += 1
    return counter - 1


def draw_gif():
    directorypath = "./data/output/"
    counter = png_file_counter(directorypath)
    image_list = list()
    for step in range(counter):
        image_list.append(Image.open(directorypath + str(step) + '.png'))
    image_list.save(directorypath + 'test.gif', save_all=True, append_images=image_list, duration=200, loop=0)

    os.remove(directorypath + '*.png')
