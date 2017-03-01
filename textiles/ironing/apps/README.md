# Ironing step-by-step process

This software has several, independent components to run when trying to iron a garment.

## Kinect Fusion
The first step is to obtain a color mesh with PCL's KinFu. It is highly recommended to run KinFu in a new dedicated folder. This step has three sub-steps:

1.Scan the garment with KinectFusion 
```bash
$ pcl_kinfu_largeScale -r -et --volume_size 1.5
```
2.Process the scan output to a mesh
```bash
$ pcl_kinfu_largeScale_mesh_output world.pcd --volume_size 1.5
```
3.Copy all image files from the `KinfuSnapshots`folder to the folder containing the `mesh_1.ply` file.

4.Process the mesh to add texture information (this will open a graphic window and wait for user input):
```bash
$ pcl_kinfu_largeScale_texture_output mesh_1.ply
```

## Mesh to color Point Cloud conversion
As the algorithm relies on color, the obtained mesh has to be converted to a colored point cloud. Current approach is to use Meshlab. 

Current approach with Meshlab is the following:

1. Load .obj file
2. View>Show Layer Dialog
3. Right-click on the textured_mesh.obj entry and select "Duplicate Current Layer"
4. Filters>Texture>Texture to Vertex Color (to transfer texture from one mesh to the other)
5. Right-click on the entry of the mesh that didn't receive the color transfer, and "Delete Current Mesh"
6. Select the remaining mesh and export as .ply

## RunIroning.py
The whole algorithm is contained in a script named RunIroning.py. This script has to be configured by adding the route of the PCL-based compiled binaries to the variable `pcl_processing_folder`. Then, it can be run by running:

```bash
$ python RunIroning.py --debug <colored_point_cloud_file>
```

For example:

```bash
$ python RunIroning.py --debug ~/Research/datasets/2017-02-13-ironing-experiments/polo-01/textured_mesh.ply
```

The `--debug` flag can be omitted if the step-by-step windows are not desired.
