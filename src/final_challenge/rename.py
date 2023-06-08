import sys
path = "/home/estebanp/Downloads/Destructores/DB fotos/amarillo2"

# rename all files to add a 2 to the end of the name

import os
for filename in os.listdir(path):
    if not filename.endswith(".jpg"):
        continue
    old_name = filename
    new_name = filename[:-4] + "_2" + filename[-4:]
    os.rename(os.path.join(path, old_name), os.path.join(path, new_name))
