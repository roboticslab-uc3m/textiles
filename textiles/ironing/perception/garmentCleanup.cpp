/*
 * GarmentCleanup
 * --------------------------------------
 *
 * After segmentation, this cleanup routine, temporarily duplicated from
 * GarmentSegmentation, has to be called to complete segmentation.
 *
 */

#include <iostream>
#include <pcl/console/parse.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/point_types.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/project_inliers.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/common/transforms.h>
#include <pcl/filters/passthrough.h>
#include <pcl/search/search.h>
#include <pcl/search/kdtree.h>
#include <pcl/segmentation/region_growing_rgb.h>
#include <pcl/point_types_conversion.h>
#include <pcl/features/moment_of_inertia_estimation.h>

#include "Debug.hpp"

void show_usage(char * program_name)
{
    std::cout << std::endl;
    std::cout << "Usage: " << program_name << " cloud_filename.[pcd|ply]" << std::endl;
    std::cout << "-h:  Show this help." << std::endl;
    std::cout << "--ransac-threshold: Set ransac threshold value (default: 0.02)" << std::endl;
    std::cout << "--hsv-s-threshold: threshold for saturation channel on hsv (default: ??)" << std::endl;
    std::cout << "--hsv-v-threshold: threshold for value channel on hsv (default: ??)" << std::endl;
}

void record_transformation(std::string output_file, Eigen::Affine3f translation_transform, Eigen::Quaternionf rotation_quaternion)
{
    std::ofstream file(output_file.c_str());
    Eigen::Transform<float, 3, Eigen::Affine> t(rotation_quaternion * translation_transform);
    file << "# Transformation Matrix:" << std::endl;
    file << t.matrix() << std::endl;
    file.close();
}

int main (int argc, char** argv)
{
    //---------------------------------------------------------------------------------------------------
    //-- Initialization stuff
    //---------------------------------------------------------------------------------------------------

    //-- Command-line arguments
    float ransac_threshold = 0.02;
    float hsv_s_threshold = 0.30;
    float hsv_v_threshold = 0.35;

    //-- Show usage
    if (pcl::console::find_switch(argc, argv, "-h") || pcl::console::find_switch(argc, argv, "--help"))
    {
        show_usage(argv[0]);
        return 0;
    }

    if (pcl::console::find_switch(argc, argv, "--ransac-threshold"))
        pcl::console::parse_argument(argc, argv, "--ransac-threshold", ransac_threshold);
    else
    {
        std::cerr << "RANSAC theshold not specified, using default value..." << std::endl;
    }

    if (pcl::console::find_switch(argc, argv, "--hsv-s-threshold"))
        pcl::console::parse_argument(argc, argv, "--hsv-s-threshold", hsv_s_threshold);
    else
    {
        std::cerr << "Saturation theshold not specified, using default value..." << std::endl;
    }

    if (pcl::console::find_switch(argc, argv, "--hsv-v-threshold"))
        pcl::console::parse_argument(argc, argv, "--hsv-v-threshold", hsv_v_threshold);
    else
    {
        std::cerr << "Value theshold not specified, using default value..." << std::endl;
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

    //-- Load point cloud data (with color)
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr source_cloud_color(new pcl::PointCloud<pcl::PointXYZRGB>);

    if (file_is_pcd)
    {
        if (pcl::io::loadPCDFile(argv[filenames[0]], *source_cloud_color) < 0)
        {
            std::cout << "Error loading colored point cloud " << argv[filenames[0]] << std::endl << std::endl;
            show_usage(argv[0]);
            return -1;
        }
    }
    else
    {
        if (pcl::io::loadPLYFile(argv[filenames[0]], *source_cloud_color) < 0)
        {
            std::cout << "Error loading colored point cloud " << argv[filenames[0]] << std::endl << std::endl;
            show_usage(argv[0]);
            return -1;
        }
    }

    //-- Print arguments to user
    std::cout << "Selected arguments: " << std::endl
              << "\tRANSAC threshold: " << ransac_threshold << std::endl
              << "\tColor point threshold: " << hsv_s_threshold << std::endl
              << "\tColor region threshold: " << hsv_v_threshold << std::endl;

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZ>);


    //--------------------------------------------------------------------------------------------------------
    //-- Program does actual work from here
    //--------------------------------------------------------------------------------------------------------
    Debug debug;
    debug.setAutoShow(false);
    debug.setEnabled(false);

    debug.setEnabled(true);
    debug.plotPointCloud<pcl::PointXYZRGB>(source_cloud_color, Debug::COLOR_ORIGINAL);
    debug.show("Original with color");

    //-- Euclidean Clustering of the resultant cloud
    pcl::search::KdTree<pcl::PointXYZRGB>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZRGB>);
    tree->setInputCloud(source_cloud_color);

    std::vector<pcl::PointIndices> cluster_indices;
    pcl::EuclideanClusterExtraction<pcl::PointXYZRGB> euclidean_custering;
    euclidean_custering.setClusterTolerance(0.005);
    euclidean_custering.setMinClusterSize(100);
    euclidean_custering.setSearchMethod(tree);
    euclidean_custering.setInputCloud(source_cloud_color);
    euclidean_custering.extract(cluster_indices);

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr largest_color_cluster(new pcl::PointCloud<pcl::PointXYZRGB>);
    int largest_cluster_size = 0;
    for (auto it = cluster_indices.begin (); it != cluster_indices.end (); ++it)
    {
      pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_cluster (new pcl::PointCloud<pcl::PointXYZRGB>);
      for (auto pit = it->indices.begin (); pit != it->indices.end (); ++pit)
        cloud_cluster->points.push_back(source_cloud_color->points[*pit]);
      cloud_cluster->width = cloud_cluster->points.size ();
      cloud_cluster->height = 1;
      cloud_cluster->is_dense = true;

      std::cout << "Found cluster of " << cloud_cluster->points.size() << " points." << std::endl;
      if (cloud_cluster->points.size() > largest_cluster_size)
      {
          largest_cluster_size = cloud_cluster->points.size();
          *largest_color_cluster = *cloud_cluster;
      }
    }

    debug.setEnabled(true);
    debug.plotPointCloud<pcl::PointXYZRGB>(largest_color_cluster, Debug::COLOR_GREEN);
    debug.show("Filtered garment cloud");

    //-- Centering the point cloud before saving it
    //-----------------------------------------------------------------------------------
    //-- Find bounding box
    pcl::MomentOfInertiaEstimation<pcl::PointXYZRGB> feature_extractor;
    pcl::PointXYZRGB min_point_AABB, max_point_AABB;
    pcl::PointXYZRGB min_point_OBB,  max_point_OBB;
    pcl::PointXYZRGB position_OBB;
    Eigen::Matrix3f rotational_matrix_OBB;

    feature_extractor.setInputCloud(largest_color_cluster);
    feature_extractor.compute();
    feature_extractor.getAABB(min_point_AABB, max_point_AABB);
    feature_extractor.getOBB(min_point_OBB, max_point_OBB, position_OBB, rotational_matrix_OBB);

    //-- Translating to center
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr centered_garment_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
    Eigen::Affine3f garment_translation_transform = Eigen::Affine3f::Identity();
    garment_translation_transform.translation() << -position_OBB.x, -position_OBB.y, -position_OBB.z;
    pcl::transformPointCloud(*largest_color_cluster, *centered_garment_cloud, garment_translation_transform);

    //-- Orient using the principal axes of the bounding box
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr oriented_garment_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
    Eigen::Vector3f principal_axis_x(max_point_OBB.x - min_point_OBB.x, 0, 0);
    Eigen::Quaternionf garment_rotation_quaternion = Eigen::Quaternionf().setFromTwoVectors(principal_axis_x, Eigen::Vector3f::UnitX()); //-- This transformation is wrong (I guess)
    Eigen::Transform<float, 3, Eigen::Affine> t2 = Eigen::Transform<float, 3, Eigen::Affine>::Identity();
    t2.rotate(rotational_matrix_OBB.inverse());
    //pcl::transformPointCloud(*centered_garment_cloud, *oriented_garment_cloud, Eigen::Vector3f(0,0,0), garment_rotation_quaternion);
    pcl::transformPointCloud(*centered_garment_cloud, *oriented_garment_cloud, t2);

    //-- Save to file
    record_transformation(argv[filenames[0]]+std::string("-transform2.txt"), garment_translation_transform, Eigen::Quaternionf(t2.rotation()));


    debug.setEnabled(true);
    debug.plotPointCloud<pcl::PointXYZRGB>(oriented_garment_cloud, Debug::COLOR_GREEN);
    debug.plotBoundingBox(min_point_OBB, max_point_OBB, position_OBB, rotational_matrix_OBB, Debug::COLOR_YELLOW);
    debug.show("Oriented garment patch");

    //-- Save point cloud in file to process it in Python
    pcl::io::savePCDFileBinary(argv[filenames[0]]+std::string("-output.pcd"), *oriented_garment_cloud);

    return 0;
}
