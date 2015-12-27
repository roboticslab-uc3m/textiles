/*******************************************************
 *  render2png
 * ----------------------------------------------------
 *
 * Obtain a 2D representation of whatever the viewer is
 * outputting in a png file
 *
 * *****************************************************/
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
    std::cout << "-o:  Output file PNG" << std::endl;
    std::cout << "-cam: Camera configuration / file" << std::endl;
}

int main(int argc, char* argv[])
{
    //-- Default arguments
    std::string output_file = "render.png";

    //-- Show usage
    if (pcl::console::find_switch(argc, argv, "-h") || pcl::console::find_switch(argc, argv, "--help"))
    {
        show_usage(argv[0]);
        return 0;
    }

    if (pcl::console::find_switch(argc, argv, "-o"))
        pcl::console::parse_argument(argc, argv, "-o", output_file);

    //-- Get point cloud file from arguments
    std::vector<int> filenames;

    filenames = pcl::console::parse_file_extension_argument(argc, argv, ".pcd");

    if (filenames.size() < 1)
    {
        show_usage(argv[0]);
        return -1;
    }

    //-- Visualization Setup
    pcl::visualization::PCLVisualizer viewer(argc, argv, "render2png");
    viewer.addCoordinateSystem(0.5, "cloud", 0);
    viewer.setBackgroundColor(0.05, 0.05, 0.05, 0);
    viewer.setShowFPS(false);
    if (!viewer.cameraParamsSet() && !viewer.cameraFileLoaded())
    {
        viewer.setCameraPosition(-0.277017, -1.53399, 1.72656,  0.176012, 0.717815, 0.673618);
    }

    //-- Add point clouds
    pcl::PCDReader pcd;
    pcl::PCLPointCloud2::Ptr cloud;
    pcl::visualization::PointCloudColorHandler<pcl::PCLPointCloud2>::Ptr color_handler;

    for (size_t i = 0; i < filenames.size(); i++)
    {
        //-- Load point cloud
        cloud.reset(new pcl::PCLPointCloud2);
        Eigen::Vector4f origin;
        Eigen::Quaternionf orientation;
        int version;
        std::cout << "Loading: " << argv[filenames[i]] << std::endl;
        if (pcd.read(argv[filenames[i]], *cloud, origin, orientation, version) < 0)
        {
            std::cerr << "Error ocurred, exiting..." << std::endl;
            return -1;
        }

        //-- Select cloud color or random color
        color_handler.reset(new pcl::visualization::PointCloudColorHandlerRandom<pcl::PCLPointCloud2>(cloud));
        for(size_t j = 0; j < cloud->fields.size(); j++)
        {
            if (cloud->fields[j].name == "rgb" || cloud->fields[j].name == "rgba")
            {
                color_handler.reset(new pcl::visualization::PointCloudColorHandlerRGBField<pcl::PCLPointCloud2>(cloud));
                break;
            }
        }


        //-- Add point cloud
        std::stringstream cloud_name;
        cloud_name << argv[filenames[i]] << "-" << i;
        viewer.addPointCloud(cloud, color_handler, origin, orientation, cloud_name.str().c_str());
        viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, cloud_name.str().c_str());

        if (i==0 && !viewer.cameraParamsSet() && !viewer.cameraFileLoaded())
        {
            viewer.resetCameraViewpoint(cloud_name.str());
            viewer.resetCamera();
        }
    }


    //-- Generate render and save
    viewer.saveScreenshot(output_file);

    return 0;
}
