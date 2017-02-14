"""
Generate data
------------------------
Compute the Kinfu meshes from Kinfu raw point clouds in batch

Usage: generate_data.py <directory>
"""

import os
import subprocess
import logging

import begin

@begin.start(auto_convert=True)
@begin.logging
def main(data_directory: 'Directory containing Kinfu raw data'='.',
         volume_size: 'Volume size used to capture the data'=1.5,
         texture: 'Generate textured mesh'=True):
    """Processes raw KinFu data to get meshes"""
    current_dir = os.path.abspath(os.path.expanduser(data_directory))

    os.chdir(current_dir)

    for directory in [i for i in os.listdir(current_dir) if os.path.isdir(i) and not i.startswith('.')]:
        logging.info('Generating data for ' + directory)
        os.chdir(directory)

        logging.info('\tGenerate mesh...')
        args = ['pcl_kinfu_largeScale_mesh_output', 'world.pcd', '--volume_size', str(volume_size)]
        p = subprocess.Popen(args, stdout=subprocess.PIPE, stderr=subprocess.PIPE)
        out, err = p.communicate()

        if texture:
            logging.info('\tGenerate textured mesh...')
            args = ['pcl_kinfu_largeScale_texture_output', 'mesh_1.ply']
            p = subprocess.Popen(args, stdout=subprocess.PIPE, stderr=subprocess.PIPE, stdin=subprocess.PIPE)
            out, err = p.communicate(b'q\r')

        logging.info(out, err)

        os.chdir('..')
