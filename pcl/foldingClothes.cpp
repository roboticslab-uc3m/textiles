#include <iostream>

#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/point_cloud.h>
#include <pcl/console/parse.h>
#include <pcl/common/transforms.h>
#include <pcl/visualization/pcl_visualizer.h>
//-- Normals
#include <pcl/point_types.h>
#include <pcl/features/normal_3d.h>
#include <pcl/filters/filter.h>
//-- Contours
#include <pcl/filters/extract_indices.h>
//-- Octree
#include <pcl/octree/octree.h>
//-- Projection
#include <pcl/ModelCoefficients.h>
#include <pcl/filters/project_inliers.h>
//-- Bounding box
#include <pcl/features/moment_of_inertia_estimation.h>
#include <vector>
#include <pcl/visualization/cloud_viewer.h>
//-- RSD estimation
#include <pcl/features/normal_3d.h>
#include <pcl/features/rsd.h>

//-- My classes
#include "PointCloudPreprocessor.hpp"
#include "ZBufferDepthImageCreator.hpp"

#include <fstream>

void show_usage(char * program_name)
{
  std::cout << std::endl;
  std::cout << "Usage: " << program_name << " cloud_filename.[pcd|ply]" << std::endl;
  std::cout << "-h:  Show this help." << std::endl;
  std::cout << "-b, --background:  Background points size (default: 1)" << std::endl;
  std::cout << "-f, --foreground:  Foreground points size (default: 5)" << std::endl;
  std::cout << "-t, --threshold:  Distance threshold for RANSAC (default: 0.03)" << std::endl;
  std::cout << "--TSDF: Input cloud is a TSDF cloud, params are cube and voxel dimensions (Default: 3m, 512 voxels)" << std::endl;
}

int main(int argc, char* argv[])
{
    //-- Main program parameters (default values)
    int viewer_point_size_background = 1;
    int viewer_point_size_foreground = 5;
    double threshold = 0.03;
    bool TSDF_enable_scale = false;
    int TSDF_cube_dimensions = 3; //-- In meters
    int TSDF_voxels = 512;

    //-- Show usage
    if (pcl::console::find_switch(argc, argv, "-h") || pcl::console::find_switch(argc, argv, "--help"))
    {
        show_usage(argv[0]);
        return 0;
    }

    //-- Check for parameters in arguments
    if (pcl::console::find_switch(argc, argv, "-b"))
        pcl::console::parse_argument(argc, argv, "-b", viewer_point_size_background);
    else if (pcl::console::find_switch(argc, argv, "--background"))
        pcl::console::parse_argument(argc, argv, "--background", viewer_point_size_background);

    if (pcl::console::find_switch(argc, argv, "-f"))
        pcl::console::parse_argument(argc, argv, "-f", viewer_point_size_foreground);
    else if (pcl::console::find_switch(argc, argv, "--foreground"))
        pcl::console::parse_argument(argc, argv, "--foreground", viewer_point_size_foreground);

    if (pcl::console::find_switch(argc, argv, "-t"))
        pcl::console::parse_argument(argc, argv, "-t", threshold);
    else if (pcl::console::find_switch(argc, argv, "--threshold"))
        pcl::console::parse_argument(argc, argv, "--threshold", threshold);

    if (pcl::console::find_switch(argc, argv, "--TSDF"))
    {
        TSDF_enable_scale = true;
        pcl::console::parse_2x_arguments(argc, argv, "--TSDF", TSDF_cube_dimensions, TSDF_voxels);
    }

    //-- Get point cloud file from arguments
    std::vector<int> filenames;
    bool file_is_pcd = false;

    filenames = pcl::console::parse_file_extension_argument(argc, argv, ".ply");

    if (filenames.size() != 1)
    {
        filenames = pcl::console::parse_file_extension_argument(argc, argv, ".pcd");

        if (filenames.size() != 1)
        {
            show_usage(argv[0]);
            return -1;
        }

        file_is_pcd = true;
    }

    //-- Print arguments to user
    std::cout << "Selected arguments: " << std::endl
              << "\tBackground point size: " << viewer_point_size_background << std::endl
              << "\tForeground point size: " << viewer_point_size_foreground << std::endl
              << "\tAngular threshold (degrees): " << threshold << std::endl
              << "\tInput file: " << argv[filenames[0]] << std::endl;

    if (TSDF_enable_scale)
        std::cout << "\tTSDF enabled with params: " << TSDF_cube_dimensions << " meters/side, "
                  << TSDF_voxels << " voxels" << std::endl;

    //-- Load point cloud data
    pcl::PointCloud<pcl::PointXYZ>::Ptr source_cloud(new pcl::PointCloud<pcl::PointXYZ>);

    if (file_is_pcd)
    {
        if (pcl::io::loadPCDFile(argv[filenames[0]], *source_cloud) < 0)
        {
            std::cout << "Error loading point cloud " << argv[filenames[0]] << std::endl << std::endl;
            show_usage(argv[0]);
            return -1;
        }
    }
    else
    {
        if (pcl::io::loadPLYFile(argv[filenames[0]], *source_cloud) < 0)
        {
            std::cout << "Error loading point cloud " << argv[filenames[0]] << std::endl << std::endl;
            show_usage(argv[0]);
            return -1;
        }
    }

    /********************************************************************************************
    * Stuff goes on here
    *********************************************************************************************/
    //-- Initial pre-processing of the mesh
    //------------------------------------------------------------------------------------
    pcl::PointCloud<pcl::PointXYZ>::Ptr garment_points(new pcl::PointCloud<pcl::PointXYZ>);
    PointCloudPreprocessor<pcl::PointXYZ> preprocessor;
    preprocessor.setScalingTSDF(TSDF_enable_scale);
    preprocessor.setScalingTSDFParams(TSDF_cube_dimensions, TSDF_voxels);
    preprocessor.setRANSACThresholdDistance(threshold);
    preprocessor.setInputCloud(source_cloud);
    preprocessor.process(*garment_points);

    //-- Find bounding box (not really required)
    pcl::MomentOfInertiaEstimation<pcl::PointXYZ> feature_extractor;
    pcl::PointXYZ min_point_AABB, max_point_AABB;
    pcl::PointXYZ min_point_OBB,  max_point_OBB;
    pcl::PointXYZ position_OBB;
    Eigen::Matrix3f rotational_matrix_OBB;

    feature_extractor.setInputCloud(garment_points);
    feature_extractor.compute();
    feature_extractor.getAABB(min_point_AABB, max_point_AABB);
    feature_extractor.getOBB(min_point_OBB, max_point_OBB, position_OBB, rotational_matrix_OBB);

    //-- Curvature stuff
    //-----------------------------------------------------------------------------------
    // Object for storing the normals.
    pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);
    // Object for storing the RSD descriptors for each point.
    pcl::PointCloud<pcl::PrincipalRadiiRSD>::Ptr descriptors(new pcl::PointCloud<pcl::PrincipalRadiiRSD>());


    // Note: you would usually perform downsampling now. It has been omitted here
    // for simplicity, but be aware that computation can take a long time.

    // Estimate the normals.
    pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> normalEstimation;
    normalEstimation.setInputCloud(garment_points);
    normalEstimation.setRadiusSearch(0.03);
    pcl::search::KdTree<pcl::PointXYZ>::Ptr kdtree(new pcl::search::KdTree<pcl::PointXYZ>);
    normalEstimation.setSearchMethod(kdtree);
    normalEstimation.compute(*normals);

    // RSD estimation object.
    pcl::RSDEstimation<pcl::PointXYZ, pcl::Normal, pcl::PrincipalRadiiRSD> rsd;
    rsd.setInputCloud(garment_points);
    rsd.setInputNormals(normals);
    rsd.setSearchMethod(kdtree);
    // Search radius, to look for neighbors. Note: the value given here has to be
    // larger than the radius used to estimate the normals.
    rsd.setRadiusSearch(0.05);
    // Plane radius. Any radius larger than this is considered infinite (a plane).
    rsd.setPlaneRadius(0.1);
    // Do we want to save the full distance-angle histograms?
    rsd.setSaveHistograms(false);

    rsd.compute(*descriptors);

    //-- Save to mat file
    std::ofstream rsd_file("rsd_data.m");
    for (int i = 0; i < garment_points->points.size(); i++)
    {
        rsd_file << garment_points->points[i].x << " "
                 << garment_points->points[i].y << " "
                 << garment_points->points[i].z << " "
                 << descriptors->points[i].r_min << " "
                 << descriptors->points[i].r_max << "\n";
    }
    rsd_file.close();

    //-- Obtain range image
    //-----------------------------------------------------------------------------------
    ZBufferDepthImageCreator<pcl::PointXYZ> depth_image_creator;
    depth_image_creator.setInputPointCloud(garment_points);
    depth_image_creator.setResolution(1024);
    depth_image_creator.setUpsampling(true);
    depth_image_creator.compute();
    Eigen::MatrixXf image = depth_image_creator.getDepthImageAsMatrix();

    //-- Temporal fix to get image (through file)
    std::ofstream file("depth_image2.m");
    file << image;
    file.close();


    /********************************************************************************************************
     * Visualization
     * *****************************************************************************************************/
    //-- Visualization Setup
    pcl::visualization::PCLVisualizer viewer("Folding clothes");
    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> red_color_handler(source_cloud, 255, 0, 0);
    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> green_color_handler(source_cloud, 0, 255, 0);
    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> blue_color_handler(source_cloud, 0, 0, 255);
    viewer.addCoordinateSystem(1.0, "origin", 0);
    viewer.setBackgroundColor(0.05, 0.05, 0.05, 0);

    //-- Add point cloud
    viewer.addPointCloud(garment_points, red_color_handler, "oriented_point_cloud");
    viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "oriented_point_cloud");

    //-- Add plane inliers
    //pcl::PointCloud<pcl::PointXYZ>::Ptr plane_points(new pcl::PointCloud<pcl::PointXYZ>(*source_cloud, table_plane_points->indices));
    //viewer.addPointCloud(plane_points, green_color_handler, "plane_point_cloud");
    //viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "plane_point_cloud");

    //-- Add normals
    //viewer.addPointCloudNormals<pcl::PointXYZ, pcl::Normal> (source_cloud, cloud_normals, 1, 0.8, "normals");
    //viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_COLOR, 0, 0, 255, "normals");

    //-- View AABB
    viewer.addCube(min_point_AABB.x, max_point_AABB.x, min_point_AABB.y, max_point_AABB.y, min_point_AABB.z, max_point_AABB.z, 1.0, 1.0, 0.0, "AABB");

    //-- View OBB
    Eigen::Vector3f position(position_OBB.x, position_OBB.y, position_OBB.z);
    Eigen::Quaternionf quat_view(rotational_matrix_OBB);
    //viewer.addCube(position, quat_view, max_point_OBB.x - min_point_OBB.x, max_point_OBB.y - min_point_OBB.y, max_point_OBB.z - min_point_OBB.z, "OBB");

    //viewer.addLine(pcl::PointXYZ(0,0,0), position_OBB, "line");
    //viewer.addLine(pcl::PointXYZ(0,0,0), pcl::PointXYZ(mass_center[0], mass_center[1], mass_center[2]), "line2");
    //viewer.addLine(pcl::PointXYZ(0,0,0), pcl::PointXYZ(30*normal_vector[0], 30*normal_vector[1], 30*normal_vector[2]), "normal");


    //-- Visualization thread
    while(!viewer.wasStopped())
    {
        viewer.spinOnce();
    }

    return 0;
}
