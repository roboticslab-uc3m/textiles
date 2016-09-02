/*
 * kinfuUnfolding
 * --------------------------------------
 *
 * Adapts unfolding code to work with kinfu 3D reconstructions
 *
 */

#include <iostream>
#include <fstream>

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
//-- Octree
#include <pcl/octree/octree.h>
#include <pcl/octree/octree_search.h>

//--OpenCV
#include <opencv2/opencv.hpp>
#include <opencv2/core/eigen.hpp>

//-- Textiles headers
#include "Debug.hpp"
#include "RGBDImageCreator.hpp"

void show_usage(char * program_name)
{
    std::cout << std::endl;
    std::cout << "Usage: " << program_name << " cloud_filename.[pcd|ply]" << std::endl;
    std::cout << "-h:  Show this help." << std::endl;
    std::cout << "--ransac-threshold: Set ransac threshold value (default: 0.02)" << std::endl;
}

void record_transformation(std::string output_file, Eigen::Transform<float, 3, Eigen::Affine> t)
{
    std::ofstream file(output_file.c_str());
    file << "# Transformation Matrix:" << std::endl;
    file << t.matrix() << std::endl;
    file.close();
}


template<typename PointT>
void record_point(std::string output_file, PointT point)
{
    std::ofstream file(output_file.c_str());
    file << point.x << " " << point.y << " " << point.z;
    file.close();
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

    debug.setEnabled(true);
    debug.plotPointCloud<pcl::PointXYZRGB>(not_table_points, Debug::COLOR_ORIGINAL);
    debug.show("Not table points");

    //-- Compute largest cluster (the garment)
    //-----------------------------------------------------------------------------------
    pcl::search::KdTree<pcl::PointXYZRGB>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZRGB>);
    tree->setInputCloud(not_table_points);

    std::vector<pcl::PointIndices> cluster_indices;
    pcl::EuclideanClusterExtraction<pcl::PointXYZRGB> euclidean_custering;
    euclidean_custering.setClusterTolerance(0.015);
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
    pcl::PointXYZRGB min_point_OBB,  max_point_OBB;
    pcl::PointXYZRGB position_OBB;
    Eigen::Matrix3f rotational_matrix_OBB;

    feature_extractor.setInputCloud(largest_cluster);
    feature_extractor.compute();
    feature_extractor.getOBB(min_point_OBB, max_point_OBB, position_OBB, rotational_matrix_OBB);


    //-- Save 2D image origin point
    record_point(argv[filenames[0]]+std::string("-origin.txt"), pcl::PointXYZ(min_point_OBB.x, max_point_OBB.y, 0));

    debug.setEnabled(false);
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
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr oriented_garment(new pcl::PointCloud<pcl::PointXYZRGB>);

    //-- Compute translation to center
    Eigen::Affine3f translation_transform = Eigen::Affine3f::Identity();
    translation_transform.translation() << -projected_center.x, -projected_center.y, -projected_center.z;

    //-- Compute rotation to orient the cloud upwards (in Z)
    Eigen::Quaternionf rotation_quaternion = Eigen::Quaternionf(Eigen::AngleAxisf(M_PI, Eigen::Vector3f::UnitX()));

    //-- Transform cloud
    Eigen::Transform<float, 3, Eigen::Affine> T(rotation_quaternion*rotational_matrix_OBB.inverse()*translation_transform);
    pcl::transformPointCloud(*source_cloud, *oriented_cloud, T);
    pcl::transformPointCloud(*largest_cluster, *oriented_garment, T);

    //-- Save to file
    record_transformation(argv[filenames[0]]+std::string("-transform.txt"), T);

    debug.setEnabled(true);
    debug.plotPointCloud<pcl::PointXYZRGB>(oriented_garment, Debug::COLOR_ORIGINAL);
    debug.plotPointCloud<pcl::PointXYZRGB>(largest_cluster, Debug::COLOR_ORIGINAL);
    debug.plotBoundingBox(min_point_OBB, max_point_OBB, position_OBB, rotational_matrix_OBB, Debug::COLOR_YELLOW);
    debug.plotBoundingBox(min_point_OBB, max_point_OBB, pcl::PointXYZRGB(0,0,0), Eigen::Matrix3f::Identity(), Debug::COLOR_BLUE);
    debug.getRawViewer()->addLine (pcl::PointXYZ(0,0,0), projected_center, 1.0, 0.0, 0.0, "line");
    debug.show("Oriented garment patch");

    //---------------------------------------------------------------------------------------------------------
    //-- RBGD data extraction from point cloud
    //---------------------------------------------------------------------------------------------------------
    //-- Required parameters
    float average_point_distance=0.005; //-- Parameter to determine output image resolution
    float lowest_height_limit = min_point_OBB.z;

    //-- Calculate image resolution
    /* Note: if not using std::abs, floating abs function seems to be
     * not supported :-/ */
    float OBB_width = std::abs(max_point_OBB.x - min_point_OBB.x);
    float OBB_height = std::abs(max_point_OBB.y - min_point_OBB.y);

    int width = std::ceil(OBB_width / average_point_distance);
    int height = std::ceil(OBB_height / average_point_distance);
    std::cout << "Creating 2D image with resolution: " << width << "x" << height << "px" << std::endl;

//    //-- Matrices to store image data
//    Eigen::MatrixXf depth = Eigen::MatrixXf::Constant(height, width, lowest_height_limit);

//    //-- Filter for points within limits:
//    pcl::octree::OctreePointCloudSearch<pcl::PointXYZRGB> octree(0.001); //-- Resolution of the src point cloud ~1mm
//    std::vector<int> points_within_bounding_box;
//    Eigen::Vector3f min_bb(min_point_OBB.x, min_point_OBB.y, lowest_height_limit);
//    Eigen::Vector3f max_bb(max_point_OBB.x, max_point_OBB.y, 1);
//    octree.setInputCloud(oriented_cloud->makeShared());
//    octree.addPointsFromInputCloud();
//    octree.boxSearch(min_bb, max_bb, points_within_bounding_box);

//    //-- Loop through those points to get depth data
//    #pragma omp parallel for
//    for (int j = 0; j < points_within_bounding_box.size(); j++)
//    {
//        int i = points_within_bounding_box[j];

//        if (isnan(oriented_cloud->points[i].x) || isnan(oriented_cloud->points[i].y ))
//            continue;

//        int index_x = (oriented_cloud->points[i].x-min_point_OBB.x) / average_point_distance;
//        int index_y = (max_point_OBB.y - oriented_cloud->points[i].y) / average_point_distance;

//        if (index_x >= width) index_x = width-1;
//        if (index_y >= height) index_y = height-1;

//        //-- ZBuffer depth map output image
//        float old_z;
//        #pragma omp critical
//        {
//            old_z = depth(index_y, index_x);
//            if (oriented_cloud->points[i].z > old_z)
//                depth(index_y, index_x) = oriented_cloud->points[i].z;
//        }
//    }



    //-- Get color images
    RGBDImageCreator<pcl::PointXYZRGB> imageCreator;
    imageCreator.setInputPointCloud(oriented_cloud);
    imageCreator.setAvgPointDist(average_point_distance);
    imageCreator.setBoundingBox(min_point_OBB, max_point_OBB);
    imageCreator.compute();
    Eigen::MatrixXf red = imageCreator.getChannelAsMatrix(RGBDImageCreator<pcl::PointXYZRGB>::CHANNEL_R);
    Eigen::MatrixXf green = imageCreator.getChannelAsMatrix(RGBDImageCreator<pcl::PointXYZRGB>::CHANNEL_G);
    Eigen::MatrixXf blue = imageCreator.getChannelAsMatrix(RGBDImageCreator<pcl::PointXYZRGB>::CHANNEL_B);

    //-- Temporal fix to get red channel image (through file)
    std::ofstream red_file((argv[filenames[0]]+std::string("-red.txt")).c_str());
    red_file << red;
    red_file.close();

    //-- Temporal fix to get green channel image (through file)
    std::ofstream green_file((argv[filenames[0]]+std::string("-green.txt")).c_str());
    green_file << green;
    green_file.close();

    //-- Temporal fix to get blue channel image (through file)
    std::ofstream blue_file((argv[filenames[0]]+std::string("-blue.txt")).c_str());
    blue_file << blue;
    blue_file.close();

    Eigen::MatrixXf depth = imageCreator.getDepthImageAsMatrix();

    //-- Temporal fix to get depth image (through file)
    std::ofstream file((argv[filenames[0]]+std::string("-depth.txt")).c_str());
    file << depth;
    file.close();


    //-- Eigen to OpenCV to save RGB image as image (Quick and dirty)
    //------------------------------------------------------------------------------
    cv::Mat image(height, width, CV_8UC3);
    uint8_t* image_ptr = (uint8_t*)image.data;

    for (int i = 0; i < height; i++)
        for (int j = 0; j < width; j++)
        {
            image_ptr[i*image.cols*3 + j*3 + 0] = blue(i, j); // B
            image_ptr[i*image.cols*3 + j*3 + 1] = green(i, j);// G
            image_ptr[i*image.cols*3 + j*3 + 2] = red(i, j);  // R
        }

    cv::imwrite(argv[filenames[0]]+std::string("-RGB.png"), image);


    //-- Obtain a mask from garment data
    //------------------------------------------------------------------------------
    //-- Mask with segmented garment data
    Eigen::MatrixXd mask = Eigen::MatrixXd::Zero(height, width);
    #pragma omp parallel for
    for (int i = 0; i < oriented_garment->points.size(); i++)
    {
        if (isnan(oriented_garment->points[i].x) || isnan(oriented_garment->points[i].y ))
            continue;

        int index_x = (oriented_garment->points[i].x-min_point_OBB.x) / average_point_distance;
        int index_y = (max_point_OBB.y - oriented_garment->points[i].y) / average_point_distance;

        if (index_x >= width) index_x = width-1;
        if (index_y >= height) index_y = height-1;

        #pragma omp critical
        {
            //-- Mask
            mask(index_y, index_x) = 255;
        }
    }


    //-- Eigen to OpenCV to save RGB image as image (Quick and dirty)
    cv::Mat mask_image(height, width, CV_8UC1);
    uint8_t* mask_image_ptr = (uint8_t*)mask_image.data;

    for (int i = 0; i < height; i++)
        for (int j = 0; j < width; j++)
            mask_image_ptr[i*mask_image.cols + j] = mask(i, j);

    cv::imwrite(argv[filenames[0]]+std::string("-mask.png"), mask_image);

    return 0;
}
