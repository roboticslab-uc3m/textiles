/*
 * wrinkleDetection
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
#include <pcl/filters/extract_indices.h>
#include <pcl/search/kdtree.h>
#include <pcl/point_types_conversion.h>
#include <pcl/features/normal_3d.h>
#include <pcl/features/normal_3d_omp.h>
#include <pcl/filters/filter.h>
#include <pcl/features/rsd.h>
#include <pcl/features/moment_of_inertia_estimation.h>

#include "Debug.hpp"

void show_usage(char * program_name)
{
    std::cout << std::endl;
    std::cout << "Usage: " << program_name << " cloud_filename.[pcd|ply]" << std::endl;
    std::cout << "-h:  Show this help." << std::endl;
    std::cout << "--normal-threshold: Set normal threshold value (default: ??)" << std::endl;
    std::cout << "--rsd: Enable RSD descriptors calculation" << std::endl;
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
    //-- Fixed arguments (to be command-line arguments)
    std::string output_image = "-depth_image.m";
    std::string output_wild = "-wild_image.m";
    std::string output_mask = "-image_mask.m";
    std::string output_rsd = "-rsd.m";

    //-- Command-line arguments
    float normal_threshold = 0.02;
    bool rsd = false;

    //-- Show usage
    if (pcl::console::find_switch(argc, argv, "-h") || pcl::console::find_switch(argc, argv, "--help"))
    {
        show_usage(argv[0]);
        return 0;
    }

    if (pcl::console::find_switch(argc, argv, "--normal-threshold"))
        pcl::console::parse_argument(argc, argv, "--normal-threshold", normal_threshold);
    else
    {
        std::cerr << "Normal theshold not specified, using default value..." << std::endl;
    }

    if (pcl::console::find_switch(argc, argv, "--rsd"))
    {
        rsd = true;
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
    std::cout << "Selected arguments: " << std::endl
              << "\tNormal threshold: " << normal_threshold << std::endl;
    if (rsd)
        std::cout << "\tRSD computation enabled" << std::endl;

    //--------------------------------------------------------------------------------------------------------
    //-- Program does actual work from here
    //--------------------------------------------------------------------------------------------------------
    Debug debug;
    debug.setAutoShow(false);
    debug.setEnabled(false);

    debug.setEnabled(false);
    debug.plotPointCloud<pcl::PointXYZRGB>(source_cloud, Debug::COLOR_ORIGINAL);
    debug.show("Original with color");

    //-- Find normals
    pcl::NormalEstimationOMP<pcl::PointXYZRGB, pcl::Normal> normalEstimator;
    pcl::search::KdTree<pcl::PointXYZRGB>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZRGB>);
    pcl::PointCloud<pcl::Normal>::Ptr cloud_normals(new pcl::PointCloud<pcl::Normal>);

    normalEstimator.setInputCloud(source_cloud);
    normalEstimator.setSearchMethod(tree);
    normalEstimator.setRadiusSearch(normal_threshold);
    normalEstimator.setViewPoint(0,0,1000);
    normalEstimator.compute(*cloud_normals);
    std::cout << "Found: " << cloud_normals->size() << " normals." << std::endl;
    pcl::PointCloud<pcl::Normal>::Ptr filtered_normals(new pcl::PointCloud<pcl::Normal>);
    std::vector<int> new_ordering;
    pcl::removeNaNNormalsFromPointCloud(*cloud_normals, *filtered_normals, new_ordering);
    std::cout << "After NaN removal: " << filtered_normals->size() << " normals." << std::endl;

    debug.setEnabled(false);
    debug.plotPointCloud<pcl::PointXYZRGB>(source_cloud, Debug::COLOR_ORIGINAL);
    debug.plotNormals<pcl::PointXYZRGB, pcl::Normal>(source_cloud, cloud_normals, Debug::COLOR_ORIGINAL,
                                                     100, 0.01);
    debug.show("Normals");

    // RSD estimation object
    if (rsd)
    {
        pcl::PointCloud<pcl::PrincipalRadiiRSD>::Ptr descriptors(new pcl::PointCloud<pcl::PrincipalRadiiRSD>());
        pcl::RSDEstimation<pcl::PointXYZRGB, pcl::Normal, pcl::PrincipalRadiiRSD> rsd;
        rsd.setInputCloud(source_cloud);
        rsd.setInputNormals(filtered_normals);
        rsd.setSearchMethod(tree);
        // Search radius, to look for neighbors. Note: the value given here has to be
        // larger than the radius used to estimate the normals.
        rsd.setRadiusSearch(0.03);
        // Plane radius. Any radius larger than this is considered infinite (a plane).
        rsd.setPlaneRadius(0.1);
        // Do we want to save the full distance-angle histograms?
        rsd.setSaveHistograms(false);

        rsd.compute(*descriptors);

        //-- Save to mat file
        std::ofstream rsd_file((argv[filenames[0]]+output_rsd).c_str());
        for (int i = 0; i < source_cloud->points.size(); i++)
        {
            rsd_file << source_cloud->points[i].x << " "
                     << source_cloud->points[i].y << " "
                     << source_cloud->points[i].z << " "
                     << descriptors->points[i].r_min << " "
                     << descriptors->points[i].r_max << "\n";
        }
        rsd_file.close();
    }

    //-- WILD
    //---------------------
    //-- Compute wild descriptor
    const double threshold = 0.03;
    std::vector<double> wild(source_cloud->points.size());

    #pragma omp parallel for
    for (int i = 0; i < source_cloud->points.size(); i++)
    {
        //-- Create vector from normal of current point
        Eigen::Vector3f current_normal(cloud_normals->points[i].normal_x,
                                       cloud_normals->points[i].normal_y,
                                       cloud_normals->points[i].normal_z);

        //-- Find neighbors within radius
        std::vector<int> pointIdxRadiusSearch;
        std::vector<float> pointRadiusSquaredDistance;
        double current_wild = -Eigen::Infinity; //-- Will this work?

        if (tree->radiusSearch(source_cloud->points[i], threshold, pointIdxRadiusSearch, pointRadiusSquaredDistance) > 0)
        {
            for (auto& j: pointIdxRadiusSearch)
            {
                //-- Compute descriptor
                Eigen::Vector3f neighbor_normal(cloud_normals->points[j].normal_x,
                                                cloud_normals->points[j].normal_y,
                                                cloud_normals->points[j].normal_z);

                current_wild += current_normal.dot(neighbor_normal);
            }

            current_wild /= (double)pointIdxRadiusSearch.size();
        }

        wild[i] =current_wild;
    }

    //-- Save to mat file
//    std::ofstream wild_file("wild_descriptors.m");
//    for (int i = 0; i < source_cloud->points.size(); i++)
//    {
//        wild_file << source_cloud->points[i].x << " "
//                 << source_cloud->points[i].y << " "
//                 << source_cloud->points[i].z << " "
//                 << wild[i] << "\n";
//    }
//    wild_file.close();


    //-- Create 2D output image
    //-------------------------------------------------------------------------------------------
    float average_point_distance=0.005; //-- Parameter to determine output image resolution

    //-- Find bounding box of input point_cloud (already centered)
    pcl::MomentOfInertiaEstimation<pcl::PointXYZRGB> feature_extractor;
    pcl::PointXYZRGB min_point_AABB, max_point_AABB;

    feature_extractor.setInputCloud(source_cloud);
    feature_extractor.compute();
    feature_extractor.getAABB(min_point_AABB, max_point_AABB);

    //-- Save 2D image origin point
    record_point(argv[filenames[0]]+std::string("-origin.txt"), pcl::PointXYZ(min_point_AABB.x, max_point_AABB.y, 0));

    //-- Calculate image resolution
    /* Note: if not using std::abs, floating abs function seems to be
     * not supported :-/ */
    float AABB_width = std::abs(max_point_AABB.x - min_point_AABB.x);
    float AABB_height = std::abs(max_point_AABB.y - min_point_AABB.y);

    int width = std::ceil(AABB_width / average_point_distance);
    int height = std::ceil(AABB_height / average_point_distance);
    std::cout << "Creating 2D image with resolution: " << width << "x" << height << "px" << std::endl;

    //-- Fill bins with z values

    Eigen::MatrixXf image = Eigen::MatrixXf::Zero(height, width);
    Eigen::MatrixXf depth = Eigen::MatrixXf::Zero(height, width);
    Eigen::MatrixXd mask = Eigen::MatrixXd::Zero(height, width);
    Eigen::MatrixXf element_count = Eigen::MatrixXf::Zero(height, width);

    #pragma omp parallel for
    for (int i = 0; i < source_cloud->points.size(); i++)
    {
        if (isnan(source_cloud->points[i].x) || isnan(source_cloud->points[i].y ))
            continue;

        int index_x = (source_cloud->points[i].x-min_point_AABB.x) / average_point_distance;
        int index_y = (max_point_AABB.y - source_cloud->points[i].y) / average_point_distance;

        if (index_x >= width) index_x = width-1;
        if (index_y >= height) index_y = height-1;

        //-- This code is for ZBuffer depth map output image
        float old_z;
        #pragma omp critical
        {
        old_z = depth(index_y, index_x);
        if (source_cloud->points[i].z > old_z)
            depth(index_y, index_x) = source_cloud->points[i].z;
        }

        /* //-- This code is for wild output image (highest value is stored)
        float old_wild;
        #pragma omp critical
        {
        old_wild = image(index_y, index_x);
        if (wild[i] > old_wild)
            image(index_y, index_x) = wild[i];
        }*/

        float old_mean;
        int n_current_bin;
        #pragma omp critical
        {
        old_mean = image(index_y, index_x);
        n_current_bin = element_count(index_y, index_x) +1;

        //-- Compute new value
        float mean = old_mean + (wild[i] - old_mean) / (float)n_current_bin;

        //-- Store back values
        image(index_y, index_x) = mean;
        element_count(index_y, index_x) = n_current_bin;

        //-- Mask
        mask(index_y, index_x) = 1;
        }

    }

    //-- Temporal fix to get depth image (through file)
    std::ofstream file((argv[filenames[0]]+output_image).c_str());
    file << depth;
    file.close();

    //-- Temporal fix to get image (through file)
    std::ofstream wild_file((argv[filenames[0]]+output_wild).c_str());
    wild_file << image;
    wild_file.close();

    //-- Temporal fix to get mask (through file)
    std::ofstream mask_file((argv[filenames[0]]+output_mask).c_str());
    mask_file << mask;
    mask_file.close();

    return 0;
}
