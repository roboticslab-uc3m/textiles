import glob, os

__author__ = 'def'

def load_data(root_path):
    good_images = []
    good_depth_files = []

    images = glob.glob(os.path.join(root_path, '*.ppm'))
    depth_files = glob.glob(os.path.join(root_path, '*.mat'))

    for image in images:
        name, ext = os.path.splitext(image)
        found = False
        for depth_file in depth_files:
            if depth_file.find(name) != -1:
                found = True
                break

        if found:
            good_images.append(image)
            good_depth_files.append(name+'.mat')

    return good_images, good_depth_files
