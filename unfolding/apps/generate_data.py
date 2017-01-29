import os, sys
import subprocess


if __name__ == '__main__':
    if len(sys.argv) == 2:
        current_dir = sys.argv[1]
    else:
        current_dir = '.'

    os.chdir(current_dir)

    for directory in [i for i in os.listdir(current_dir) if os.path.isdir(i)]:
        print("Generating data for " + directory)
        os.chdir(directory)

        print("\tGenerate mesh...")
        args = ['pcl_kinfu_largeScale_mesh_output', 'world.pcd',
                '--volumen_size', str(1.5)]
        p = subprocess.Popen(args, stdout=subprocess.PIPE, stderr=subprocess.PIPE)
        out, err = p.communicate()

#        print "\tGenerate textured mesh..."
#        args = ['pcl_kinfu_largeScale_texture_output', os.path.join(directory,'mesh_1.ply')]
#        p = subprocess.Popen(args, stdout=subprocess.PIPE, stderr=subprocess.PIPE, stdin=subprocess.PIPE)
#        out, err = p.communicate("q\r")
#        print out, err

        os.chdir('..')

    
