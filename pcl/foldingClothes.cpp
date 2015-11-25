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
//-- Plane fitting
#include <pcl/ModelCoefficients.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
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
//#include <boost/thread/thread.hpp>
//-- Passthrough filter
#include <pcl/filters/passthrough.h>
//-- Range images
#include <pcl/range_image/range_image.h>
#include <pcl/visualization/range_image_visualizer.h>

#include "ZBufferDepthImageCreator.hpp"

#include <fstream>

void show_usage(char * program_name)
{
  std::cout << std::endl;
  std::cout << "Usage: " << program_name << " cloud_filename.[pcd|ply]" << std::endl;
  std::cout << "-h:  Show this help." << std::endl;
  std::cout << "-b, --background:  Background points size (default: 1)" << std::endl;
  std::cout << "-f, --foreground:  Foreground points size (default: 5)" << std::endl;
  std::cout << "-t, --threshold:  Angular threshold for contour points (default: 5º)" << std::endl;
}

int main(int argc, char* argv[])
{
    //-- Main program parameters (default values)
    int viewer_point_size_background = 1;
    int viewer_point_size_foreground = 5;
    double threshold = 5; //-- In degrees

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
    //-- Find table's plane
    //------------------------------------------------------------------------------------
    pcl::ModelCoefficients::Ptr table_plane_coefficients(new pcl::ModelCoefficients);
    pcl::PointIndices::Ptr table_plane_points(new pcl::PointIndices);
    pcl::SACSegmentation<pcl::PointXYZ> segmentation;
    segmentation.setOptimizeCoefficients(true);
    segmentation.setModelType(pcl::SACMODEL_PLANE);
    segmentation.setMethodType(pcl::SAC_RANSAC);
    segmentation.setDistanceThreshold(5);
    segmentation.setInputCloud(source_cloud);
    segmentation.segment(*table_plane_points, *table_plane_coefficients);

    if (table_plane_points->indices.size() == 0)
    {
        std::cerr << "Could not estimate a planar model for input point cloud." << std::endl;
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
    //-- Get points from the contour
    pcl::PointCloud<pcl::PointXYZ>::Ptr not_table_points (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::ExtractIndices<pcl::PointXYZ> extract_indices;
    extract_indices.setInputCloud(source_cloud);
    extract_indices.setIndices(table_plane_points);
    extract_indices.setNegative(true);
    extract_indices.filter(*not_table_points);

    //-- Find bounding box:
    //-----------------------------------------------------------------------------------
    pcl::MomentOfInertiaEstimation <pcl::PointXYZ> feature_extractor;
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
    pcl::PointCloud<pcl::PointXYZ>::Ptr centered_cloud (new pcl::PointCloud<pcl::PointXYZ> ());
    Eigen::Affine3f translation_transform = Eigen::Affine3f::Identity();
    translation_transform.translation() << -position_OBB.x, -position_OBB.y, -position_OBB.z;
    pcl::transformPointCloud (*not_table_points, *centered_cloud, translation_transform);

    //-- Orient using the plane normal
    pcl::PointCloud<pcl::PointXYZ>::Ptr oriented_cloud (new pcl::PointCloud<pcl::PointXYZ> ());
    Eigen::Vector3f normal_vector(table_plane_coefficients->values[0], table_plane_coefficients->values[1], table_plane_coefficients->values[2]);
    Eigen::Quaternionf rotation_quaternion = Eigen::Quaternionf().setFromTwoVectors(normal_vector, Eigen::Vector3f::UnitZ());
    pcl::transformPointCloud(*centered_cloud, *oriented_cloud, Eigen::Vector3f(0,0,0), rotation_quaternion);

    //-- Remove negative outliers (table noise)
    pcl::PointCloud<pcl::PointXYZ>::Ptr garment_points (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PassThrough<pcl::PointXYZ> passthrough_filter;
    passthrough_filter.setInputCloud(oriented_cloud);
    passthrough_filter.setFilterFieldName("z");
    passthrough_filter.setFilterLimits(0.0, FLT_MAX);
    passthrough_filter.setFilterLimitsNegative(false);
    passthrough_filter.filter(*garment_points);

    //-- Find bounding box
    feature_extractor.setInputCloud(garment_points);
    feature_extractor.compute();
    feature_extractor.getAABB(min_point_AABB, max_point_AABB);
    feature_extractor.getOBB(min_point_OBB, max_point_OBB, position_OBB, rotational_matrix_OBB);

    //-- Obtain range image
    //-----------------------------------------------------------------------------------
    ZBufferDepthImageCreator<pcl::PointXYZ> depth_image_creator;
    depth_image_creator.setInputPointCloud(garment_points);
    depth_image_creator.setResolution(320);
    depth_image_creator.compute();
    Eigen::MatrixXf image = depth_image_creator.getDepthImageAsMatrix();

    //-- Temporal fix to get image (through file)
    std::ofstream file("depth_image.m");
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
    viewer.addCoordinateSystem(50.0, "origin", 0);
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
