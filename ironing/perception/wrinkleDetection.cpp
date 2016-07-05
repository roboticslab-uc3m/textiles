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
#include <pcl/features/principal_curvatures.h>

#include "Debug.hpp"

void show_usage(char * program_name)
{
    std::cout << std::endl;
    std::cout << "Usage: " << program_name << " cloud_filename.[pcd|ply]" << std::endl;
    std::cout << "-h:  Show this help." << std::endl;
    std::cout << "--normal-threshold: Set normal threshold value (default: ??)" << std::endl;
}

int main (int argc, char** argv)
{
    //---------------------------------------------------------------------------------------------------
    //-- Initialization stuff
    //---------------------------------------------------------------------------------------------------

    //-- Command-line arguments
    float normal_threshold = 0.02;

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
    normalEstimator.compute(*cloud_normals);
    std::cout << "Found: " << cloud_normals->size() << " normals." << std::endl;
    pcl::PointCloud<pcl::Normal>::Ptr filtered_normals(new pcl::PointCloud<pcl::Normal>);
    std::vector<int> new_ordering;
    pcl::removeNaNNormalsFromPointCloud(*cloud_normals, *filtered_normals, new_ordering);
    std::cout << "After NaN removal: " << filtered_normals->size() << " normals." << std::endl;

    debug.setEnabled(true);
    debug.plotPointCloud<pcl::PointXYZRGB>(source_cloud, Debug::COLOR_ORIGINAL);
    debug.plotNormals<pcl::PointXYZRGB, pcl::Normal>(source_cloud, cloud_normals, Debug::COLOR_ORIGINAL,
                                                     100, 0.01);
    debug.show("Normals");

    //-- Principal curvatures
    pcl::PointCloud<pcl::PrincipalCurvatures>::Ptr principal_curvatures(new pcl::PointCloud<pcl::PrincipalCurvatures>);

    pcl::PrincipalCurvaturesEstimation<pcl::PointXYZRGB, pcl::Normal, pcl::PrincipalCurvatures> principal_curvatures_estimation;
    principal_curvatures_estimation.setInputCloud(source_cloud);
    principal_curvatures_estimation.setInputNormals(cloud_normals);
    principal_curvatures_estimation.setSearchMethod(tree);
    principal_curvatures_estimation.setRadiusSearch(0.1);
    principal_curvatures_estimation.compute(*principal_curvatures);

    debug.setEnabled(true);
    debug.getRawViewer()->addPointCloudPrincipalCurvatures<pcl::PointXYZRGB, pcl::Normal>(source_cloud, cloud_normals, principal_curvatures);
    debug.show("Principal Curvatures");
    return 0;
}
