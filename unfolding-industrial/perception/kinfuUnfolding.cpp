/*
 * kinfuUnfolding
 * --------------------------------------
 *
 * Adapts unfolding code to work with kinfu 3D reconstructions
 *
 */

#include <iostream>

//-- PCL basics & io
#include <pcl/console/parse.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/point_types.h>
//-- Downsampling
#include <pcl/filters/voxel_grid.h>
//-- Plane fitting
#include <pcl/ModelCoefficients.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
//-- Filter by indices
#include <pcl/filters/extract_indices.h>
//-- Euclidean clustering
#include <pcl/search/kdtree.h>
#include <pcl/segmentation/extract_clusters.h>
//-- Bounding box
#include <pcl/features/moment_of_inertia_estimation.h>
//-- Point projection
#include <pcl/filters/project_inliers.h>
//-- Transformations
#include <pcl/common/transforms.h>

//-- Textiles headers
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

    //-- Load point cloud data (with color)
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr source_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);

    if (file_is_pcd)
    {
        if (pcl::io::loadPCDFile(argv[filenames[0]], *source_cloud) < 0)
        {
            std::cout << "Error loading colored point cloud " << argv[filenames[0]] << std::endl << std::endl;
            show_usage(argv[0]);
            return -1;
        }
    }
    else
    {
        if (pcl::io::loadPLYFile(argv[filenames[0]], *source_cloud) < 0)
        {
            std::cout << "Error loading colored point cloud " << argv[filenames[0]] << std::endl << std::endl;
            show_usage(argv[0]);
            return -1;
        }
    }

    //-- Print arguments to user
    std::cout << "Selected arguments: " << std::endl;
    std::cout << "\tRANSAC threshold: " << ransac_threshold << std::endl;


    //-- Setup debug object
    Debug debug;
    debug.setAutoShow(false);
    debug.setEnabled(false);

    //--------------------------------------------------------------------------------------------------------
    //-- Program does actual work from here
    //--------------------------------------------------------------------------------------------------------
    debug.setEnabled(true);
    debug.plotPointCloud<pcl::PointXYZRGB>(source_cloud, Debug::COLOR_ORIGINAL);
    debug.show("Original with color");

    //-- Downsample the dataset prior to plane detection (using a leaf size of 1cm)
    //-----------------------------------------------------------------------------------
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_downsampled(new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::VoxelGrid<pcl::PointXYZRGB> voxel_grid;
    voxel_grid.setInputCloud(source_cloud);
    voxel_grid.setLeafSize(0.01f, 0.01f, 0.01f);
    voxel_grid.filter(*cloud_downsampled);
    std::cout << "Initially PointCloud has: " << source_cloud->points.size ()  << " data points." << std::endl;
    std::cout << "PointCloud after filtering has: " << cloud_downsampled->points.size ()  << " data points." << std::endl;

    //-- Find table's plane
    //------------------------------------------------------------------------------------
    pcl::ModelCoefficients::Ptr table_plane_coefficients(new pcl::ModelCoefficients);
    pcl::PointIndices::Ptr table_plane_points(new pcl::PointIndices);
    pcl::SACSegmentation<pcl::PointXYZRGB> ransac_segmentation;
    ransac_segmentation.setOptimizeCoefficients(true);
    ransac_segmentation.setModelType(pcl::SACMODEL_PLANE);
    ransac_segmentation.setMethodType(pcl::SAC_RANSAC);
    ransac_segmentation.setDistanceThreshold(ransac_threshold);
    ransac_segmentation.setInputCloud(cloud_downsampled);
    ransac_segmentation.segment(*table_plane_points, *table_plane_coefficients);

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

    debug.setEnabled(false);
    debug.plotPointCloud<pcl::PointXYZRGB>(source_cloud, Debug::COLOR_ORIGINAL);
    debug.plotPlane(*table_plane_coefficients, Debug::COLOR_BLUE);
    debug.show("Table plane");

    //-- Find points that do not belong to the plane
    //----------------------------------------------------------------------------------
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr not_table_points (new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::ExtractIndices<pcl::PointXYZRGB> extract_indices;
    extract_indices.setInputCloud(cloud_downsampled);
    extract_indices.setIndices(table_plane_points);
    extract_indices.setNegative(true);
    extract_indices.filter(*not_table_points);

    debug.setEnabled(false);
    debug.plotPointCloud<pcl::PointXYZRGB>(not_table_points, Debug::COLOR_ORIGINAL);
    debug.show("Not table points");

    //-- Compute largest cluster (the garment)
    //-----------------------------------------------------------------------------------
    pcl::search::KdTree<pcl::PointXYZRGB>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZRGB>);
    tree->setInputCloud(not_table_points);

    std::vector<pcl::PointIndices> cluster_indices;
    pcl::EuclideanClusterExtraction<pcl::PointXYZRGB> euclidean_custering;
    euclidean_custering.setClusterTolerance(0.01);
    euclidean_custering.setMinClusterSize(100);
    euclidean_custering.setSearchMethod(tree);
    euclidean_custering.setInputCloud(not_table_points);
    euclidean_custering.extract(cluster_indices);

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr largest_cluster(new pcl::PointCloud<pcl::PointXYZRGB>);
    int largest_cluster_size = 0;
    for (auto it = cluster_indices.begin (); it != cluster_indices.end (); ++it)
    {
      pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_cluster (new pcl::PointCloud<pcl::PointXYZRGB>);
      for (auto pit = it->indices.begin (); pit != it->indices.end (); ++pit)
        cloud_cluster->points.push_back(not_table_points->points[*pit]);
      cloud_cluster->width = cloud_cluster->points.size ();
      cloud_cluster->height = 1;
      cloud_cluster->is_dense = true;

      std::cout << "Found cluster of " << cloud_cluster->points.size() << " points." << std::endl;
      if (cloud_cluster->points.size() > largest_cluster_size)
      {
          largest_cluster_size = cloud_cluster->points.size();
          *largest_cluster = *cloud_cluster;
      }
    }

    debug.setEnabled(false);
    debug.plotPointCloud<pcl::PointXYZRGB>(largest_cluster, Debug::COLOR_ORIGINAL);
    debug.show("Filtered garment cloud");

    //-- Find bounding box:
    //-----------------------------------------------------------------------------------
    pcl::MomentOfInertiaEstimation<pcl::PointXYZRGB> feature_extractor;
    pcl::PointXYZRGB min_point_AABB, max_point_AABB;
    pcl::PointXYZRGB min_point_OBB,  max_point_OBB;
    pcl::PointXYZRGB position_OBB;
    Eigen::Matrix3f rotational_matrix_OBB;

    feature_extractor.setInputCloud(largest_cluster);
    feature_extractor.compute();
    feature_extractor.getAABB(min_point_AABB, max_point_AABB);
    feature_extractor.getOBB(min_point_OBB, max_point_OBB, position_OBB, rotational_matrix_OBB);

    debug.setEnabled(true);
    debug.plotPointCloud<pcl::PointXYZRGB>(largest_cluster, Debug::COLOR_ORIGINAL);
    debug.plotBoundingBox(min_point_OBB, max_point_OBB, position_OBB, rotational_matrix_OBB, Debug::COLOR_GREEN);
    debug.show("Oriented bounding cloud");

    //-- Project bounding box center point on table plane
    //------------------------------------------------------------------------------------
    //-- Project center point onto given plane:
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr center_to_be_projected_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
    center_to_be_projected_cloud->points.push_back(position_OBB);
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr center_projected_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);

    pcl::ProjectInliers<pcl::PointXYZRGB> project_inliners;
    project_inliners.setModelType(pcl::SACMODEL_PLANE);
    project_inliners.setInputCloud(center_to_be_projected_cloud);
    project_inliners.setModelCoefficients(table_plane_coefficients);
    project_inliners.filter(*center_projected_cloud);
    pcl::PointXYZRGB projected_center = center_projected_cloud->points[0];

    //-- Center and orient it at the origin
    //------------------------------------------------------------------------------------
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr oriented_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);

    //-- Compute translation to center
    Eigen::Affine3f translation_transform = Eigen::Affine3f::Identity();
    translation_transform.translation() << -projected_center.x, -projected_center.y, -projected_center.z;

    //-- Compute rotation using the the plane normal
    Eigen::Vector3f normal_vector(table_plane_coefficients->values[0], table_plane_coefficients->values[1], table_plane_coefficients->values[2]);
    //-- Check normal vector orientation
    if (normal_vector.dot(Eigen::Vector3f::UnitZ()) >= 0)
        normal_vector = -normal_vector;
    Eigen::Quaternionf rotation_quaternion = Eigen::Quaternionf().setFromTwoVectors(normal_vector, Eigen::Vector3f::UnitZ());
    Eigen::Transform<float, 3, Eigen::Affine> T(rotation_quaternion * translation_transform);
    //T.rotate(rotational_matrix_OBB.inverse());

    //-- Transform cloud
    pcl::transformPointCloud(*largest_cluster, *oriented_cloud, T);

    debug.setEnabled(true);
    debug.plotPointCloud<pcl::PointXYZRGB>(oriented_cloud, Debug::COLOR_ORIGINAL);
    debug.plotBoundingBox(min_point_OBB, max_point_OBB, position_OBB, rotational_matrix_OBB, Debug::COLOR_YELLOW);
    debug.getRawViewer()->addLine (pcl::PointXYZ(0,0,0), projected_center, 1.0, 0.0, 0.0, "line");
    debug.show("Oriented garment patch");
}
