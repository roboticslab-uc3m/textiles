#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/console/parse.h>
#include <pcl/visualization/pcl_visualizer.h>
//-- RSD estimation
#include <pcl/features/normal_3d.h>
#include <pcl/features/normal_3d_omp.h>
#include <pcl/features/rsd.h>

#include <fstream>

void show_usage(char * program_name)
{
  std::cout << std::endl;
  std::cout << "Usage: " << program_name << std::endl;
  std::cout << "-h:  Show this help." << std::endl;
  std::cout << "--side: Side of the cube (mm)" << std::endl;
  std::cout << "--samples: Number of points / side" << std::endl;
  std::cout << "-o: Output file" << std::endl;
}

int main (int argc, char** argv)
{
    //-- input data
    float side = 1;
    int samples = 50;
    std::string filename = "cube.m";

    //-- Show usage
    if (pcl::console::find_switch(argc, argv, "-h") || pcl::console::find_switch(argc, argv, "--help"))
    {
        show_usage(argv[0]);
        return 0;
    }

    //-- Read from terminal arguments
    if (pcl::console::find_switch(argc, argv, "--side"))
        pcl::console::parse_argument(argc, argv, "--side", side);

    if (pcl::console::find_switch(argc, argv, "--samples"))
        pcl::console::parse_argument(argc, argv, "--samples", samples);

    if (pcl::console::find_switch(argc, argv, "-o"))
        pcl::console::parse_argument(argc, argv, "-o", filename);

    //-- Fill in the cloud data
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
    //cloud->width  = pow(samples,3);
    //cloud->height = 1;
    //cloud->points.resize (cloud->width * cloud->height);
    //std::cout << "Working with " << cloud->points.size() << " points." << std::endl;


    float point_distance = side/samples;
    std::cout << "Resolution: " << point_distance << std::endl;
    std::cout << "Creating point cloud..." << std::endl;

    for (int j = 0; j < samples; j++)
        for (int i = 0; i < samples; i++)
        {
            pcl::PointXYZ p;
            p.x = i * point_distance;
            p.y = j * point_distance;
            p.z = 0;
            cloud->points.push_back(p);

            pcl::PointXYZ p2;
            p2.x = i * point_distance;
            p2.y = j * point_distance;
            p2.z = side;
            cloud->points.push_back(p2);
        }

    for (int k = 1; k < samples-1; k++)
        for (int j = 0; j < samples; j++)
            if (j == 0 || j == samples-1)
            {
                for (int i = 0; i < samples; i++)
                {
                    pcl::PointXYZ p;
                    p.x = i * point_distance;
                    p.y = j * point_distance;
                    p.z = k * point_distance;
                    cloud->points.push_back(p);
                }
            }
            else
            {
                pcl::PointXYZ p;
                p.x = 0;
                p.y = j * point_distance;
                p.z = k * point_distance;
                cloud->points.push_back(p);

                pcl::PointXYZ p2;
                p2.x = side;
                p2.y = j * point_distance;
                p2.z = k * point_distance;
                cloud->points.push_back(p2);
            }

    cloud->width = cloud->points.size();
    cloud->height = 1;

    std::cout << "Working with " << cloud->points.size() << " points." << std::endl;


    //-- Curvature stuff
    //-----------------------------------------------------------------------------------
    // Object for storing the normals.
    pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);
    // Object for storing the RSD descriptors for each point.
    pcl::PointCloud<pcl::PrincipalRadiiRSD>::Ptr descriptors(new pcl::PointCloud<pcl::PrincipalRadiiRSD>());


    // Note: you would usually perform downsampling now. It has been omitted here
    // for simplicity, but be aware that computation can take a long time.

    // Estimate the normals.
    std::cout << "Computing normals..." << std::endl;
    pcl::NormalEstimationOMP<pcl::PointXYZ, pcl::Normal> normalEstimation;
    normalEstimation.setInputCloud(cloud);
    normalEstimation.setRadiusSearch(0.005);
    pcl::search::KdTree<pcl::PointXYZ>::Ptr kdtree(new pcl::search::KdTree<pcl::PointXYZ>);
    normalEstimation.setSearchMethod(kdtree);
    normalEstimation.compute(*normals);
    std::cout << "Found " << normals->points.size() << " normals." << std::endl;

    // RSD estimation object.
    std::cout << "Computing RSD descriptors..." << std::endl;
    pcl::RSDEstimation<pcl::PointXYZ, pcl::Normal, pcl::PrincipalRadiiRSD> rsd;
    rsd.setInputCloud(cloud);
    rsd.setInputNormals(normals);
    rsd.setSearchMethod(kdtree);
    // Search radius, to look for neighbors. Note: the value given here has to be
    // larger than the radius used to estimate the normals.
    rsd.setRadiusSearch(0.007);
    // Plane radius. Any radius larger than this is considered infinite (a plane).
    rsd.setPlaneRadius(0.005);
    // Do we want to save the full distance-angle histograms?
    rsd.setSaveHistograms(false);

    rsd.compute(*descriptors);
    std::cout << "Found " << descriptors->points.size() << " descriptors." << std::endl;

    //-- Save to mat file
    std::cout << "Saving to file " << filename <<  " ..." << std::endl;
    std::ofstream rsd_file(filename.c_str());
    for (int i = 0; i < cloud->points.size(); i++)
    {
        rsd_file << cloud->points[i].x << " "
                 << cloud->points[i].y << " "
                 << cloud->points[i].z << " "
                 << descriptors->points[i].r_min << " "
                 << descriptors->points[i].r_max << "\n";
    }
    rsd_file.close();

    //-- Save point cloud
    pcl::io::savePCDFileASCII((filename+".pcd").c_str(), *cloud);

    //-- Visualization
    //--------------------------------------------------------------------------------------
    //-- Visualization Setup
    pcl::visualization::PCLVisualizer viewer("Magic cube");
    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> red_color_handler(cloud, 255, 0, 0);
    viewer.addCoordinateSystem(1.0, "origin", 0);
    viewer.setBackgroundColor(0.05, 0.05, 0.05, 0);

    //-- Add point cloud
    viewer.addPointCloud(cloud, red_color_handler, "cube");
    viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "cube");

    //-- Add normals
    viewer.addPointCloudNormals<pcl::PointXYZ, pcl::Normal> (cloud, normals, 1, 0.05, "normals");
    viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_COLOR, 0, 0, 255, "normals");


    //-- Visualization thread
    while(!viewer.wasStopped())
    {
        viewer.spinOnce();
    }

    return 0;
}
