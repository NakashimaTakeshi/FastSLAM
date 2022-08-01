import os
import glob

import matplotlib.pyplot as plt
from PIL import Image

output_directory_path = "./data/output/"
temp_output_directory_path = output_directory_path + "temp/"


def rescale_list(list_, min_value, max_value):
    return [(x - min(list_)) / (max(list_) - min(list_)) * (max_value - min_value) + min_value for x in list_]


def save_png():
    counter = file_counter(temp_output_directory_path, "png")
    plt.savefig(temp_output_directory_path + str(counter) + '.png', bbox_inches='tight')


def file_counter(directory_path, filetype):
    counter: int = 0
    while os.path.exists(directory_path + str(counter) + "." + filetype):
        counter += 1
    return counter


def draw_gif():
    n_image = file_counter(temp_output_directory_path, "png")
    n_gif = file_counter(output_directory_path, "gif")
    im = list()
    image_list = list()
    for counter in range(n_image):
        im.append(Image.open(temp_output_directory_path + str(counter) + '.png'))
        image_list.append(im[counter])
    im[0].save(output_directory_path + str(n_gif) + '.gif', save_all=True, append_images=image_list, duration=100, loop=1000)

    clear_png_output_directory()


def clear_output_directory():
    for file in glob.glob(temp_output_directory_path + '*', recursive=True):
        os.remove(file)


def clear_png_output_directory():
    for file in glob.glob(temp_output_directory_path + '*.png', recursive=True):
        os.remove(file)


def save_operation(action, counter):
    operation_file = temp_output_directory_path + 'operation_output.csv'
    with open(operation_file, 'a') as f:
        f.write(str(action) + ',' + str(counter) + '\n')
        f.close()
