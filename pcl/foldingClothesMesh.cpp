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
#include <pcl/features/normal_3d_omp.h>
#include <pcl/features/rsd.h>
//-- Voxels
#include <pcl/filters/voxel_grid.h>

//-- My classes
#include "MeshPreprocessor.hpp"

#include <fstream>

void show_usage(char * program_name)
{
  std::cout << std::endl;
  std::cout << "Usage: " << program_name << " cloud_filename.[pcd|ply]" << std::endl;
  std::cout << "-h:  Show this help." << std::endl;
  std::cout << "-t, --threshold:  Distance threshold for RANSAC (default: 0.03)" << std::endl;
  std::cout << "-r, --rsd: Output file for RSD data" << std::endl;
  std::cout << "--rsd-params: Parameters for RSD (kdtree radius for normals, for curvature and plane threshold" << std::endl;
}

int main(int argc, char* argv[])
{
    //-- Main program parameters (default values)
    double threshold = 0.03;
    std::string output_rsd_data = "rsd_data.m";
    double rsd_normal_radius = 0.05;
    double rsd_curvature_radius = 0.07;
    double rsd_plane_threshold = 0.2;

    //-- Show usage
    if (pcl::console::find_switch(argc, argv, "-h") || pcl::console::find_switch(argc, argv, "--help"))
    {
        show_usage(argv[0]);
        return 0;
    }

    //-- Check for parameters in arguments
    if (pcl::console::find_switch(argc, argv, "-t"))
        pcl::console::parse_argument(argc, argv, "-t", threshold);
    else if (pcl::console::find_switch(argc, argv, "--threshold"))
        pcl::console::parse_argument(argc, argv, "--threshold", threshold);

    if (pcl::console::find_switch(argc, argv, "-r"))
        pcl::console::parse_argument(argc, argv, "-r", output_rsd_data);
    else if (pcl::console::find_switch(argc, argv, "--rsd"))
        pcl::console::parse_argument(argc, argv, "--rsd", output_rsd_data);

    if (pcl::console::find_switch(argc, argv, "--rsd-params"))
        pcl::console::parse_3x_arguments(argc, argv, "--rsd-params", rsd_normal_radius, rsd_curvature_radius,
                                         rsd_plane_threshold);

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
              << "\tRANSAC threshold: " << threshold << std::endl
              << "\tRSD:" << std::endl
              << "\t\tFile: " << output_rsd_data << std::endl
              << "\t\tNormal search radius: " << rsd_normal_radius << std::endl
              << "\t\tCurvature search radius: " << rsd_curvature_radius << std::endl
              << "\t\tPlane threshold: " << rsd_plane_threshold << std::endl
              << "\tInput file: " << argv[filenames[0]] << std::endl;



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
    //-- Downsampling the mesh prior to RANSAC
    pcl::PointCloud<pcl::PointXYZ>::Ptr downsampled_point_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::VoxelGrid<pcl::PointXYZ> voxel_grid_filter;
    voxel_grid_filter.setInputCloud(source_cloud);
    voxel_grid_filter.setLeafSize(0.01f, 0.01f, 0.01f); //-- 1cm^3
    voxel_grid_filter.filter(*downsampled_point_cloud);

    //-- Find table's plane
    //------------------------------------------------------------------------------------
    pcl::ModelCoefficients::Ptr table_plane_coefficients(new pcl::ModelCoefficients);
    pcl::PointIndices::Ptr table_plane_points(new pcl::PointIndices);
    pcl::SACSegmentation<pcl::PointXYZ> segmentation;
    segmentation.setOptimizeCoefficients(true);
    segmentation.setModelType(pcl::SACMODEL_PLANE);
    segmentation.setMethodType(pcl::SAC_RANSAC);
    segmentation.setDistanceThreshold(threshold);
    segmentation.setInputCloud(downsampled_point_cloud);
    segmentation.segment(*table_plane_points, *table_plane_coefficients);

    if (table_plane_points->indices.size() == 0)
    {
        std::cerr << "Could not estimate a planar model for input point cloud." << std::endl;
        return false;
    }
    else
    {
        std::cout << "Plane equation: (" << table_plane_coefficients->values[0] << ") x + ("
                                         << table_plane_coefficients->values[1] << ") y + ("
                                         << table_plane_coefficients->values[2] << ") z + ("
                                         << table_plane_coefficients->values[3] << ") = 0" << std::endl;
        std::cout << "Model inliers: " << table_plane_points->indices.size() << std::endl;
    }

    //-- Find points that do not belong to the plane
    //----------------------------------------------------------------------------------
    //-- Filter table points
    pcl::PointCloud<pcl::PointXYZ>::Ptr not_table_points(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::ExtractIndices<pcl::PointXYZ> extract_indices;
    extract_indices.setInputCloud(downsampled_point_cloud);
    extract_indices.setIndices(table_plane_points);
    extract_indices.setNegative(true);
    extract_indices.filter(*not_table_points);


    //-- Find bounding box:
    //-----------------------------------------------------------------------------------
    pcl::MomentOfInertiaEstimation<pcl::PointXYZ> feature_extractor;
    pcl::PointXYZ min_point_AABB, max_point_AABB;
    pcl::PointXYZ min_point_OBB,  max_point_OBB;
    pcl::PointXYZ position_OBB;
    Eigen::Matrix3f rotational_matrix_OBB;

    feature_extractor.setInputCloud(not_table_points);
    feature_extractor.compute();
    feature_extractor.getAABB(min_point_AABB, max_point_AABB);
    feature_extractor.getOBB(min_point_OBB, max_point_OBB, position_OBB, rotational_matrix_OBB);

    //-- Transform point cloud
    //-----------------------------------------------------------------------------------
    //-- Translating to center
    pcl::PointCloud<pcl::PointXYZ>::Ptr centered_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    Eigen::Affine3f translation_transform = Eigen::Affine3f::Identity();
    translation_transform.translation() << -position_OBB.x, -position_OBB.y, -position_OBB.z;
    pcl::transformPointCloud(*not_table_points, *centered_cloud, translation_transform);

    //-- Orient using the plane normal
    pcl::PointCloud<pcl::PointXYZ>::Ptr oriented_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    Eigen::Vector3f normal_vector(table_plane_coefficients->values[0], table_plane_coefficients->values[1], table_plane_coefficients->values[2]);
    Eigen::Quaternionf rotation_quaternion = Eigen::Quaternionf().setFromTwoVectors(normal_vector, Eigen::Vector3f::UnitZ());
    pcl::transformPointCloud(*centered_cloud, *oriented_cloud, Eigen::Vector3f(0,0,0), rotation_quaternion);

    pcl::PointCloud<pcl::PointXYZ>::Ptr garment_points(new pcl::PointCloud<pcl::PointXYZ>);
    *garment_points = *oriented_cloud;

    //-- Find bounding box (not really required)
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
    pcl::NormalEstimationOMP<pcl::PointXYZ, pcl::Normal> normalEstimation;
    normalEstimation.setInputCloud(garment_points);
    normalEstimation.setRadiusSearch(rsd_normal_radius);
    pcl::search::KdTree<pcl::PointXYZ>::Ptr kdtree(new pcl::search::KdTree<pcl::PointXYZ>);
    normalEstimation.setSearchMethod(kdtree);
    normalEstimation.setViewPoint(0, 0 , 2);
    normalEstimation.compute(*normals);

    // RSD estimation object.
    pcl::RSDEstimation<pcl::PointXYZ, pcl::Normal, pcl::PrincipalRadiiRSD> rsd;
    rsd.setInputCloud(garment_points);
    rsd.setInputNormals(normals);
    rsd.setSearchMethod(kdtree);
    // Search radius, to look for neighbors. Note: the value given here has to be
    // larger than the radius used to estimate the normals.
    rsd.setRadiusSearch(rsd_curvature_radius);
    // Plane radius. Any radius larger than this is considered infinite (a plane).
    rsd.setPlaneRadius(rsd_plane_threshold);
    // Do we want to save the full distance-angle histograms?
    rsd.setSaveHistograms(false);

    rsd.compute(*descriptors);

    //-- Save to mat file
    std::ofstream rsd_file(output_rsd_data.c_str());
    for (int i = 0; i < garment_points->points.size(); i++)
    {
        rsd_file << garment_points->points[i].x << " "
                 << garment_points->points[i].y << " "
                 << garment_points->points[i].z << " "
                 << descriptors->points[i].r_min << " "
                 << descriptors->points[i].r_max << "\n";
    }
    rsd_file.close();

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
    viewer.addPointCloudNormals<pcl::PointXYZ, pcl::Normal> (garment_points, normals, 1, 0.03, "normals");
    viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_COLOR, 0, 0, 255, "normals");

    //-- View AABB
    // viewer.addCube(min_point_AABB.x, max_point_AABB.x, min_point_AABB.y, max_point_AABB.y, min_point_AABB.z, max_point_AABB.z, 1.0, 1.0, 0.0, "AABB");

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
