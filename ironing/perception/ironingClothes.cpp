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
    file << "Translation:" << std::endl;
    file << translation_transform.matrix() << std::endl << std::endl;
    file << "Rotation:" << std::endl;
    file << rotation_quaternion.toRotationMatrix() << std::endl;
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

    debug.setEnabled(false);
    debug.plotPointCloud<pcl::PointXYZRGB>(source_cloud_color, Debug::COLOR_ORIGINAL);
    debug.show("Original with color");

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
        debug.setEnabled(false);
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
        debug.setEnabled(false);
        debug.plotPlane(*garment_plane, Debug::COLOR_BLUE);
        debug.plotPointCloud<pcl::PointXYZ>(source_cloud, Debug::COLOR_RED);
        debug.show("Garment plane");
    }

    //-- Reorient cloud to origin (with color point cloud)
    //-----------------------------------------------------------------------------------
    //-- Translating to center
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr centered_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
    Eigen::Affine3f translation_transform = Eigen::Affine3f::Identity();
    translation_transform.translation() << -garment_projected_center.x, -garment_projected_center.y, -garment_projected_center.z;
    pcl::transformPointCloud(*source_cloud_color, *centered_cloud, translation_transform);

    //-- Orient using the plane normal
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr oriented_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
    Eigen::Vector3f normal_vector(garment_plane->values[0], garment_plane->values[1], garment_plane->values[2]);
    //-- Check normal vector orientation
    if (normal_vector.dot(Eigen::Vector3f::UnitZ()) >= 0 && normal_vector.dot(Eigen::Vector3f::UnitY()) >= 0)
        normal_vector = -normal_vector;
    Eigen::Quaternionf rotation_quaternion = Eigen::Quaternionf().setFromTwoVectors(normal_vector, Eigen::Vector3f::UnitZ());
    pcl::transformPointCloud(*centered_cloud, *oriented_cloud, Eigen::Vector3f(0,0,0), rotation_quaternion);

    //-- Save to file
    record_transformation(argv[filenames[0]]+std::string("-transform1.txt"), translation_transform, rotation_quaternion);

    debug.setEnabled(false);
    debug.plotPointCloud<pcl::PointXYZRGB>(oriented_cloud, Debug::COLOR_GREEN);
    debug.show("Oriented");

    //-- Filter points under the garment table
    //-----------------------------------------------------------------------------------
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr garment_table_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::PassThrough<pcl::PointXYZRGB> passthrough_filter;
    passthrough_filter.setInputCloud(oriented_cloud);
    passthrough_filter.setFilterFieldName("z");
    passthrough_filter.setFilterLimits(-ransac_threshold/2.0f, FLT_MAX);
    passthrough_filter.setFilterLimitsNegative(false);
    passthrough_filter.filter(*garment_table_cloud);

    debug.setEnabled(false);
    debug.plotPointCloud<pcl::PointXYZRGB>(garment_table_cloud, Debug::COLOR_GREEN);
    debug.show("Table cloud (filtered)");

    //-- Color segmentation of the garment
    //-----------------------------------------------------------------------------------
    //-- HSV thresholding
    pcl::PointCloud<pcl::PointXYZHSV>::Ptr hsv_cloud(new pcl::PointCloud<pcl::PointXYZHSV>);
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr filtered_garment_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::PointCloudXYZRGBtoXYZHSV(*garment_table_cloud, *hsv_cloud);
    for (int i = 0; i < hsv_cloud->points.size(); i++)
    {
        if (isnan(hsv_cloud->points[i].x) || isnan(hsv_cloud->points[i].y || isnan(hsv_cloud->points[i].z)))
            continue;
        if (hsv_cloud->points[i].s > hsv_s_threshold &&  hsv_cloud->points[i].v > hsv_v_threshold)
            filtered_garment_cloud->push_back(garment_table_cloud->points[i]);
    }

    debug.setEnabled(false);
    debug.plotPointCloud<pcl::PointXYZRGB>(filtered_garment_cloud, Debug::COLOR_GREEN);
    debug.show("Garment cloud");

    //-- Euclidean Clustering of the resultant cloud
    pcl::search::KdTree<pcl::PointXYZRGB>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZRGB>);
    tree->setInputCloud(filtered_garment_cloud);

    std::vector<pcl::PointIndices> cluster_indices;
    pcl::EuclideanClusterExtraction<pcl::PointXYZRGB> euclidean_custering;
    euclidean_custering.setClusterTolerance(0.005);
    euclidean_custering.setMinClusterSize(100);
    euclidean_custering.setSearchMethod(tree);
    euclidean_custering.setInputCloud(filtered_garment_cloud);
    euclidean_custering.extract(cluster_indices);

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr largest_color_cluster(new pcl::PointCloud<pcl::PointXYZRGB>);
    int largest_cluster_size = 0;
    for (auto it = cluster_indices.begin (); it != cluster_indices.end (); ++it)
    {
      pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_cluster (new pcl::PointCloud<pcl::PointXYZRGB>);
      for (auto pit = it->indices.begin (); pit != it->indices.end (); ++pit)
        cloud_cluster->points.push_back(filtered_garment_cloud->points[*pit]);
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

    debug.setEnabled(false);
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
    Eigen::Quaternionf garment_rotation_quaternion = Eigen::Quaternionf().setFromTwoVectors(principal_axis_x, Eigen::Vector3f::UnitX());
    pcl::transformPointCloud(*centered_garment_cloud, *oriented_garment_cloud, Eigen::Vector3f(0,0,0), garment_rotation_quaternion);

    //-- Save to file
    record_transformation(argv[filenames[0]]+std::string("-transform2.txt"), garment_translation_transform, garment_rotation_quaternion);


    debug.setEnabled(false);
    debug.plotPointCloud<pcl::PointXYZRGB>(oriented_garment_cloud, Debug::COLOR_GREEN);
    debug.plotBoundingBox(min_point_OBB, max_point_OBB, position_OBB, rotational_matrix_OBB, Debug::COLOR_YELLOW);
    debug.show("Oriented garment patch");

    //-- Save point cloud in file to process it in Python
    pcl::io::savePCDFileBinary(argv[filenames[0]]+std::string("-output.pcd"), *oriented_garment_cloud);

    return 0;
}
