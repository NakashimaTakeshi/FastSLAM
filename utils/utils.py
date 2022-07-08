import os
import glob

import matplotlib.pyplot as plt
from PIL import Image

directorypath = "./data/output/"


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
    return counter


def draw_gif():
    directorypath = "./data/output/"
    n_image = png_file_counter(directorypath)
    print(n_image)
    im = list()
    image_list = list()
    for counter in range(n_image):
        im.append(Image.open(directorypath + str(counter) + '.png'))
        image_list.append(im[counter])
    im[0].save(directorypath + '00.gif', save_all=True, append_images=image_list, duration=200, loop=0)
    # image_list.append(Image.open(directorypath + str(step) + '.png'))
    # image_list.save(directorypath + 'test.gif', save_all=True, append_images=image_list, duration=200, loop=0)

    clear_output_directory()


def clear_output_directory():
    print(directorypath)
    for file in glob.glob(directorypath + '*.png', recursive=True):
        os.remove(file)

    # im = list()  # image_list = list()  # for step in range(step):  #     im.append(Image.open(read_path + str(step + 1) + '.png'))  #     image_list.append(im[step])  # im[0].save(save_path + '00.gif', save_all=True, append_images=image_list, duration=200, loop=0)
