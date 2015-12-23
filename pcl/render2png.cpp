#include <vector>
#include <iostream>
#include <sstream>

#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/point_cloud.h>
#include <pcl/console/parse.h>
#include <pcl/visualization/pcl_visualizer.h>

void show_usage(char * program_name)
{
  std::cout << std::endl;
  std::cout << "Usage: " << program_name << " cloud_filename.[pcd|ply]" << std::endl;
  std::cout << "-h:  Show this help." << std::endl;
}

int main(int argc, char* argv[])
{
    //-- Show usage
    if (pcl::console::find_switch(argc, argv, "-h") || pcl::console::find_switch(argc, argv, "--help"))
    {
        show_usage(argv[0]);
        return 0;
    }

    //-- Get point cloud file from arguments
    std::vector<int> filenames;
    bool file_is_pcd = false;

    filenames = pcl::console::parse_file_extension_argument(argc, argv, ".ply");

    if (filenames.size() < 1)
    {
        filenames = pcl::console::parse_file_extension_argument(argc, argv, ".pcd");

        if (filenames.size() < 1)
        {
            show_usage(argv[0]);
            return -1;
        }
        else
        {
            file_is_pcd = true;
        }
    }

    //-- Load all files
    pcl::PointCloud<pcl::PointXYZ>::Ptr clouds[filenames.size()];
    for (int i = 0; i < filenames.size(); i++)
    {
        std::string current_file = argv[filenames[i]];
        clouds[i] = pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>);

        std::cout << "Loading: " << current_file << std::endl;

        if (file_is_pcd)
        {
            if (pcl::io::loadPCDFile(current_file.c_str(), *clouds[i]) < 0)
            {
                std::cout << "Error loading point cloud " << current_file.c_str() << std::endl << std::endl;
                show_usage(argv[0]);
                return -1;
            }
        }
        else
        {
            if (pcl::io::loadPLYFile(current_file.c_str(), *clouds[i]) < 0)
            {
                std::cout << "Error loading point cloud " << current_file.c_str() << std::endl << std::endl;
                show_usage(argv[0]);
                return -1;
            }
        }
    }

    //-- Visualization Setup
    pcl::visualization::PCLVisualizer viewer("render2png");
    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> red_color_handler(clouds[0], 255, 0, 0);
    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> green_color_handler(clouds[0], 0, 255, 0);
    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> blue_color_handler(clouds[0], 0, 0, 255);
    viewer.addCoordinateSystem(1.0, "cloud", 0);
    viewer.setBackgroundColor(0.05, 0.05, 0.05, 0);
    viewer.setShowFPS(false);
    viewer.setCameraPosition(-0.277017, -1.53399, 1.72656,  0.176012, 0.717815, 0.673618);


    //-- Add point cloud
    for (int i = 0; i < filenames.size(); i++)
    {
        std::stringstream ss;
        ss << "cloud" << i;
        viewer.addPointCloud(clouds[i], red_color_handler, ss.str().c_str());
        viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, ss.str().c_str());
    }

    //-- Generate render and save
    viewer.saveScreenshot("screenshot.png");

    return 0;
}
