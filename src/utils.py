from os import listdir
from os.path import isfile, join
import random

def get_image_path():
    dirpath = "res/images/rectangles/"
    o = [dirpath + f for f in listdir(dirpath) if isfile(join(dirpath, f))]
    return random.choice(o)