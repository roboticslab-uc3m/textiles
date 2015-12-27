/*
 * clusteringCleanup
 * --------------------------------------
 *
 * Takes a pcd as input and divides it into several
 * clusters
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
#include <pcl/features/normal_3d.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>

void show_usage(char * program_name)
{
    std::cout << std::endl;
    std::cout << "Usage: " << program_name << " cloud_filename.[pcd|ply]" << std::endl;
    std::cout << "-h:  Show this help." << std::endl;
    std::cout << "-o: Output folder for cluster data" << std::endl;
    std::cout << "--ransac: Enable plane segmentation" << std::endl;
    std::cout << "--ransac-threshold: Set ransac threshold value" << std::endl;
    std::cout << "--cluster-params: Set clustering parameters" << std::endl;
}

int main (int argc, char** argv)
{
    //-- Command-line arguments
    std::string output_folder = "";
    bool ransac_enabled = false;
    float ransac_threshold = 0.02;
    float cluster_tolerance = 0.02;
    float cluster_min_size = 100;
    float cluster_max_size = 25000;

    //-- Show usage
    if (pcl::console::find_switch(argc, argv, "-h") || pcl::console::find_switch(argc, argv, "--help"))
    {
        show_usage(argv[0]);
        return 0;
    }

    //-- Check for parameters in arguments
    if (pcl::console::find_switch(argc, argv, "-o"))
        pcl::console::parse_argument(argc, argv, "-o", output_folder);
    else
    {
        std::cerr << "Ouput folder not specified" << std::endl;
        show_usage(argv[0]);
        return -2;
    }

    if (pcl::console::find_switch(argc, argv, "--ransac"))
    {
        ransac_enabled = true;
        if (pcl::console::find_switch(argc, argv, "--ransac-threshold"))
            pcl::console::parse_argument(argc, argv, "--ransac-threshold", ransac_threshold);
    }

    if (pcl::console::find_switch(argc, argv, "--cluster-params"))
        pcl::console::parse_3x_arguments(argc, argv, "--cluster-params", cluster_tolerance,
                                         cluster_min_size, cluster_max_size);

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
              << "\tOutput folder: " << output_folder << std::endl
              << "\tCluster params: " << std::endl
              << "\t\tTolerance: " << cluster_tolerance << std::endl
              << "\t\tMin size: " << cluster_min_size << std::endl
              << "\t\tMax size: " << cluster_max_size << std::endl
              << "\tInput file: " << argv[filenames[0]] << std::endl;

    if (ransac_enabled)
        std::cout << "\tRANSAC enabled with threshold: " << ransac_threshold << std::endl;


    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZ>);
    if (ransac_enabled)
    {
        // Create the filtering object: downsample the dataset using a leaf size of 1cm
        pcl::VoxelGrid<pcl::PointXYZ> voxel_grid;
        voxel_grid.setInputCloud(source_cloud);
        voxel_grid.setLeafSize(0.01f, 0.01f, 0.01f);
        voxel_grid.filter(*cloud_filtered);
        std::cout << "PointCloud after filtering has: " << cloud_filtered->points.size ()  << " data points." << std::endl; //*

        // Create the segmentation object for the planar model and set all the parameters
        pcl::SACSegmentation<pcl::PointXYZ> seg;
        pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
        pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_plane(new pcl::PointCloud<pcl::PointXYZ> ());
        seg.setOptimizeCoefficients(true);
        seg.setModelType(pcl::SACMODEL_PLANE);
        seg.setMethodType(pcl::SAC_RANSAC);
        seg.setDistanceThreshold(ransac_threshold);

        int i=0, nr_points = (int) cloud_filtered->points.size();
        while (cloud_filtered->points.size() > 0.3 * nr_points)
        {
            // Segment the largest planar component from the remaining cloud
            seg.setInputCloud(cloud_filtered);
            seg.segment(*inliers, *coefficients);
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

            // Get the points associated with the planar surface
            extract.filter(*cloud_plane);
            std::cout << "PointCloud representing the planar component: " << cloud_plane->points.size () << " data points." << std::endl;

            // Remove the planar inliers, extract the rest
            extract.setNegative(true);
            extract.filter(*cloud_f);
            *cloud_filtered = *cloud_f;
        }
    }
    else
    {
        *cloud_filtered = *source_cloud;
    }

    // Creating the KdTree object for the search method of the extraction
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);
    tree->setInputCloud(cloud_filtered);

    std::vector<pcl::PointIndices> cluster_indices;
    pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
    ec.setClusterTolerance(cluster_tolerance);
    ec.setMinClusterSize(cluster_min_size);
    ec.setMaxClusterSize(cluster_max_size);
    ec.setSearchMethod(tree);
    ec.setInputCloud(cloud_filtered);
    ec.extract(cluster_indices);

    int j = 0;
    for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin(); it != cluster_indices.end (); ++it)
    {
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_cluster(new pcl::PointCloud<pcl::PointXYZ>);
        for (std::vector<int>::const_iterator pit = it->indices.begin(); pit != it->indices.end (); ++pit)
            cloud_cluster->points.push_back(cloud_filtered->points[*pit]);
        cloud_cluster->width = cloud_cluster->points.size();
        cloud_cluster->height = 1;
        cloud_cluster->is_dense = true;

        std::cout << "PointCloud representing the Cluster: " << cloud_cluster->points.size() << " data points." << std::endl;
        std::stringstream ss;
        ss << output_folder << "/" << "cloud_cluster_" << j << ".pcd";
                pcl::PCDWriter writer;
        writer.write<pcl::PointXYZ> (ss.str(), *cloud_cluster, false);
        j++;
    }

    return 0;
}
