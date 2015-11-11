#include <iostream>

#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/point_cloud.h>
#include <pcl/console/parse.h>
#include <pcl/common/transforms.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/point_types.h>
#include <pcl/features/normal_3d.h>
#include <pcl/filters/filter.h>

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

    if (filenames.size() != 1)
    {
        filenames = pcl::console::parse_file_extension_argument(argc, argv, ".pcd");

        if (filenames.size() != 1)
        {
            show_usage(argv[0]);
            return -1;
        }
        else
        {
            file_is_pcd = true;
        }
    }

    //-- Load point cloud data
    pcl::PointCloud<pcl::PointXYZ>::Ptr source_cloud(new pcl::PointCloud<pcl::PointXYZ>());

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


    //-- Stuff goes on here
    //-- Find normals
    pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> normalEstimator;
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>());
    pcl::PointCloud<pcl::Normal>::Ptr cloud_normals(new pcl::PointCloud<pcl::Normal>);

    normalEstimator.setInputCloud(source_cloud);
    normalEstimator.setSearchMethod(tree);
    normalEstimator.setRadiusSearch(3);
    normalEstimator.compute(*cloud_normals);
    std::cout << "Found: " << cloud_normals->size() << " normals." << std::endl;
    pcl::PointCloud<pcl::Normal>::Ptr filtered_normals(new pcl::PointCloud<pcl::Normal>);
    std::vector<int> new_ordering;
    pcl::removeNaNNormalsFromPointCloud(*cloud_normals, *filtered_normals, new_ordering);
    std::cout << "After NaN removal: " << filtered_normals->size() << " normals." << std::endl;

    //-- Visualization Setup
    pcl::visualization::PCLVisualizer viewer("Folding clothes");
    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> cloud_color_handler(source_cloud, 255, 0, 0);
    viewer.addPointCloud(source_cloud, cloud_color_handler, "loaded_point_cloud");
    viewer.addPointCloudNormals<pcl::PointXYZ, pcl::Normal> (source_cloud, cloud_normals, 1, 0.8, "normals");
    viewer.addCoordinateSystem(1.0, "cloud", 0);
    viewer.setBackgroundColor(0.05, 0.05, 0.05, 0);
    viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "loaded_point_cloud");
    viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_COLOR, 0, 0, 255, "normals");

    //-- Visualization thread
    while(!viewer.wasStopped())
    {
        viewer.spinOnce();
    }

    return 0;
}
