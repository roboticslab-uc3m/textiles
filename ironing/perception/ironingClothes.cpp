/*
 * ironingClothes
 * --------------------------------------
 *
 * Placeholder text
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

#include "Debug.hpp"

void show_usage(char * program_name)
{
    std::cout << std::endl;
    std::cout << "Usage: " << program_name << " cloud_filename.[pcd|ply]" << std::endl;
    std::cout << "-h:  Show this help." << std::endl;
    std::cout << "--ransac-threshold: Set ransac threshold value (default: 0.02)" << std::endl;
}

int main (int argc, char** argv)
{
    //---------------------------------------------------------------------------------------------------
    //-- Initialization stuff
    //---------------------------------------------------------------------------------------------------

    //-- Command-line arguments
    float ransac_threshold = 0.02;

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

    //-- Print arguments to user
    std::cout << "Selected arguments: " << std::endl
              << "\tRANSAC threshold: " << ransac_threshold << std::endl;

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZ>);


    //--------------------------------------------------------------------------------------------------------
    //-- Program does actual work from here
    //--------------------------------------------------------------------------------------------------------
    Debug debug;
    debug.setAutoShow(false);
    debug.setEnabled(false);

    //-- Downsample the dataset prior to plane detection (using a leaf size of 1cm)
    //-----------------------------------------------------------------------------------
    pcl::VoxelGrid<pcl::PointXYZ> voxel_grid;
    voxel_grid.setInputCloud(source_cloud);
    voxel_grid.setLeafSize(0.01f, 0.01f, 0.01f);
    voxel_grid.filter(*cloud_filtered);
    std::cout << "Initially PointCloud has: " << source_cloud->points.size ()  << " data points." << std::endl;
    std::cout << "PointCloud after filtering has: " << cloud_filtered->points.size ()  << " data points." << std::endl;

    //-- Detect all possible planes
    //-----------------------------------------------------------------------------------
    std::vector<pcl::ModelCoefficientsPtr> all_planes;

    pcl::SACSegmentation<pcl::PointXYZ> ransac_segmentation;
    ransac_segmentation.setOptimizeCoefficients(true);
    ransac_segmentation.setModelType(pcl::SACMODEL_PLANE);
    ransac_segmentation.setMethodType(pcl::SAC_RANSAC);
    ransac_segmentation.setDistanceThreshold(ransac_threshold);

    pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
    pcl::ModelCoefficients::Ptr current_plane(new pcl::ModelCoefficients);

    int i=0, nr_points = (int) cloud_filtered->points.size();
    while (cloud_filtered->points.size() > 0.3 * nr_points)
    {
        // Segment the largest planar component from the remaining cloud
        ransac_segmentation.setInputCloud(cloud_filtered);
        ransac_segmentation.segment(*inliers, *current_plane);
        if (inliers->indices.size() == 0)
        {
            std::cout << "Could not estimate a planar model for the given dataset." << std::endl;
            break;
        }

        // Extract the planar inliers from the input cloud
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_f(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::ExtractIndices<pcl::PointXYZ> extract;
        extract.setInputCloud(cloud_filtered);
        extract.setIndices(inliers);
        extract.setNegative(false);

        // Remove the planar inliers, extract the rest
        extract.setNegative(true);
        extract.filter(*cloud_f);
        *cloud_filtered = *cloud_f;

        //-- Save plane
        pcl::ModelCoefficients::Ptr copy_current_plane(new pcl::ModelCoefficients);
        *copy_current_plane = *current_plane;
        all_planes.push_back(copy_current_plane);

        //-- Debug stuff
        debug.plotPlane(*current_plane, Debug::COLOR_BLUE);
        debug.plotPointCloud<pcl::PointXYZ>(cloud_filtered, Debug::COLOR_RED);
        debug.show("Plane segmentation");
    }

    //-- Filter planes to obtain garment plane
    //-----------------------------------------------------------------------------------
    pcl::ModelCoefficients::Ptr garment_plane(new pcl::ModelCoefficients);
    float min_height = FLT_MAX;
    pcl::PointXYZ garment_projected_center;

    for(int i = 0; i < all_planes.size(); i++)
    {
        //-- Check orientation
        Eigen::Vector3f normal_vector(all_planes[i]->values[0],
                                      all_planes[i]->values[1],
                                      all_planes[i]->values[2]);
        normal_vector.normalize();
        Eigen::Vector3f good_orientation(0, -1, -1);
        good_orientation.normalize();

        std::cout << "Checking vector with dot product: " << std::abs(normal_vector.dot(good_orientation)) << std::endl;
        if (std::abs(normal_vector.dot(good_orientation)) >= 0.9)
        {
            //-- Check "height" (height is defined in the local frame of reference in the yz direction)
            //-- With this frame, it is approximately equal to the norm of the vector OO' (being O the
            //-- center of the old frame and O' the projection of that center onto the plane).

            //-- Project center point onto given plane:
            pcl::PointCloud<pcl::PointXYZ>::Ptr center_to_be_projected_cloud(new pcl::PointCloud<pcl::PointXYZ>);
            center_to_be_projected_cloud->points.push_back(pcl::PointXYZ(0,0,0));
            pcl::PointCloud<pcl::PointXYZ>::Ptr center_projected_cloud(new pcl::PointCloud<pcl::PointXYZ>);

            pcl::ProjectInliers<pcl::PointXYZ> project_inliners;
            project_inliners.setModelType(pcl::SACMODEL_PLANE);
            project_inliners.setInputCloud(center_to_be_projected_cloud);
            project_inliners.setModelCoefficients(all_planes[i]);
            project_inliners.filter(*center_projected_cloud);
            pcl::PointXYZ projected_center = center_projected_cloud->points[0];
            Eigen::Vector3f projected_center_vector(projected_center.x, projected_center.y, projected_center.z);

            float height = projected_center_vector.norm();
            if (height < min_height)
            {
                min_height = height;
                *garment_plane = *all_planes[i];
                garment_projected_center = projected_center;
            }
        }
    }

    if (!(min_height < FLT_MAX))
    {
        std::cerr << "Garment plane not found!" << std::endl;
        return -3;
    }
    else
    {
        std::cout << "Found closest plane with h=" << min_height << std::endl;

        //-- Debug stuff
        debug.setEnabled(true);
        debug.plotPlane(*garment_plane, Debug::COLOR_BLUE);
        debug.plotPointCloud<pcl::PointXYZ>(source_cloud, Debug::COLOR_RED);
        debug.show("Garment plane");
    }

    //-- Reorient cloud to origin
    //-----------------------------------------------------------------------------------
    //-- Translating to center
    pcl::PointCloud<pcl::PointXYZ>::Ptr centered_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    Eigen::Affine3f translation_transform = Eigen::Affine3f::Identity();
    translation_transform.translation() << -garment_projected_center.x, -garment_projected_center.y, -garment_projected_center.z;
    pcl::transformPointCloud(*source_cloud, *centered_cloud, translation_transform);

    //-- Orient using the plane normal
    pcl::PointCloud<pcl::PointXYZ>::Ptr oriented_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    Eigen::Vector3f normal_vector(garment_plane->values[0], garment_plane->values[1], garment_plane->values[2]);
    Eigen::Quaternionf rotation_quaternion = Eigen::Quaternionf().setFromTwoVectors(normal_vector, Eigen::Vector3f::UnitZ());
    pcl::transformPointCloud(*centered_cloud, *oriented_cloud, Eigen::Vector3f(0,0,0), rotation_quaternion);

    debug.plotPointCloud<pcl::PointXYZ>(oriented_cloud, Debug::COLOR_BLUE);
    debug.show("Oriented");

    //-- Filter points under the garment table
    //-----------------------------------------------------------------------------------
    //-- Code goes here

    //-- Filter points with clustering to remove outliers / do a color segmentation of the garment
    //-----------------------------------------------------------------------------------
    //-- Code goes here

    return 0;
}
