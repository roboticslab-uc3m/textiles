import glob, os
from collections import namedtuple

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

Result = namedtuple('Result', ['name', 'bumpiness','segmentation', 'clustering', 'pnp', ])

def load_results_data(root_path):
    results = []

    bumpiness_files = glob.glob(os.path.join(root_path, '*-bumpiness.txt'))
    segmentation_image_files = glob.glob(os.path.join(root_path, '*-segmentation.png'))
    clustering_image_files = glob.glob(os.path.join(root_path, '*-clustering.png'))
    pnp_image_files = glob.glob(os.path.join(root_path, '*-pnp.png'))

    # We are assuming there is always a segmentation file for each garment (exceptions raise in pnp stage)
    for seg_image in segmentation_image_files:
        folder = os.path.dirname(seg_image)
        name = os.path.basename(seg_image)
        garment_name = os.path.splitext(name)[0].split('-')[0]
        cluster_image = None
        pnp_image = None
        bumpiness_data = None

        if any(map(lambda x: x.find(garment_name+'-')!=-1, clustering_image_files)):
            cluster_image = os.path.join(folder, garment_name + '-clustering.png')
        if any(map(lambda x: x.find(garment_name+'-')!=-1, pnp_image_files)):
            pnp_image = os.path.join(folder, garment_name + '-pnp.png')
        if any(map(lambda x: x.find(garment_name+'-')!=-1, bumpiness_files)):
            with open(os.path.join(folder, garment_name+'-bumpiness.txt'), 'r') as f:
                bumpiness_data = [ int(line.strip('\n')) for line in f.readlines() ]

        results.append(Result(garment_name, bumpiness_data, seg_image, cluster_image, pnp_image))

    return results
