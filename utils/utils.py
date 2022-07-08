import os
import glob

import matplotlib.pyplot as plt
from PIL import Image

output_directory_path = "./data/output/"


def rescale_list(list_, min_value, max_value):
    return [(x - min(list_)) / (max(list_) - min(list_)) * (max_value - min_value) + min_value for x in list_]


def save_png():
    counter = png_file_counter(output_directory_path)
    plt.savefig(output_directory_path + str(counter) + '.png', bbox_inches='tight')


def png_file_counter(directorypath):
    counter: int = 0
    while os.path.exists(directorypath + str(counter) + ".png"):
        counter += 1
    return counter


def draw_gif():
    n_image = png_file_counter(output_directory_path)
    im = list()
    image_list = list()
    for counter in range(n_image):
        im.append(Image.open(output_directory_path + str(counter) + '.png'))
        image_list.append(im[counter])
    im[0].save(output_directory_path + 'mcl.gif', save_all=True, append_images=image_list, duration=200, loop=0)

    clear_output_directory()


def clear_output_directory():
    for file in glob.glob(output_directory_path + '*.png', recursive=True):
        os.remove(file)

    # os.remove(output_directory_path + 'mcl.gif')


def save_operation(action, counter):
    operation_file = output_directory_path + 'operation_output.csv'
    with open(operation_file, 'a') as f:
        f.write(str(action) + ',' + str(counter) + '\n')
        f.close()
