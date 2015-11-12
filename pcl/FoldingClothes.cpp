#include <iostream>

#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/point_cloud.h>
#include <pcl/console/parse.h>
#include <pcl/common/transforms.h>
#include <pcl/visualization/pcl_visualizer.h>
//-- Normals
#include <pcl/point_types.h>
#include <pcl/features/normal_3d.h>
#include <pcl/filters/filter.h>
//-- Plane fitting
#include <pcl/ModelCoefficients.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
//-- Contours
#include <pcl/filters/extract_indices.h>


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


    //-- Stuff goes on here
    //-- Find normals
    pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> normalEstimator;
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);
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

    //-- Plane fitting
    pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
    pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
    pcl::SACSegmentation<pcl::PointXYZ> segmentation;
    segmentation.setOptimizeCoefficients(true);
    segmentation.setModelType(pcl::SACMODEL_PLANE);
    segmentation.setMethodType(pcl::SAC_RANSAC);
    segmentation.setDistanceThreshold(5);
    segmentation.setInputCloud(source_cloud);
    segmentation.segment(*inliers, *coefficients);

    if (inliers->indices.size() == 0)
    {
        std::cerr << "Could not estimate a planar model for input point cloud." << std::endl;
    }
    else
    {
        std::cout << "Model coefficients: " << coefficients->values[0] << " "
                                            << coefficients->values[1] << " "
                                            << coefficients->values[2] << " "
                                            << coefficients->values[3] << std::endl;
        std::cout << "Model inliers: " << inliers->indices.size() << std::endl;
    }

    //-- Look for contour (JGVictores suggested method)
    Eigen::Vector3f plane_normal(coefficients->values[0], coefficients->values[1], coefficients->values[2]);
    double plane_normal_norm = plane_normal.norm();
    std::vector<int> contour_points_indices;

    double threshold = 5; //-- In degrees
    double lower_limit = sin(-threshold*M_PI/180.0);
    double upper_limit = sin(threshold*M_PI/180.0);
    std::cout << "With a threshold of " << threshold << " degrees, limits are: (" << lower_limit << ", " << upper_limit
              << ")" << std::endl;

    for (int i = 0; i < filtered_normals->size(); i++)
    {
        Eigen::Vector3f point_normal(filtered_normals->at(i).normal[0], filtered_normals->at(i).normal[1], filtered_normals->at(i).normal[2]);
        double point_normal_norm = point_normal.norm();

        double scalar_product = (plane_normal.dot(point_normal))/(plane_normal_norm*point_normal_norm);
        if (lower_limit <= scalar_product && scalar_product <= upper_limit )
            contour_points_indices.push_back(new_ordering[i]);
    }

    std::cout << "Obtained " << contour_points_indices.size() << " contour points." << std::endl;

    //-- Get points from the contour
    pcl::PointCloud<pcl::PointXYZ>::Ptr contour_points(new pcl::PointCloud<pcl::PointXYZ>);
    boost::shared_ptr<std::vector<int> > contour_indices (new std::vector<int> (contour_points_indices));
    pcl::ExtractIndices<pcl::PointXYZ> extract_indices;
    extract_indices.setInputCloud(source_cloud);
    extract_indices.setIndices(contour_indices);
    extract_indices.setNegative(false);
    extract_indices.filter(*contour_points);

    //-- Visualization Setup
    pcl::visualization::PCLVisualizer viewer("Folding clothes");
    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> cloud_color_handler(source_cloud, 255, 0, 0);
    viewer.addCoordinateSystem(1.0, "cloud", 0);
    viewer.setBackgroundColor(0.05, 0.05, 0.05, 0);

    //-- Add point cloud
    viewer.addPointCloud(source_cloud, cloud_color_handler, "loaded_point_cloud");
    viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "loaded_point_cloud");

    //-- Add plane inliers
    pcl::PointCloud<pcl::PointXYZ>::Ptr plane_points(new pcl::PointCloud<pcl::PointXYZ>(*source_cloud, inliers->indices));
    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> plane_color_handler(plane_points, 0, 255, 0);
    viewer.addPointCloud(plane_points, plane_color_handler, "plane_point_cloud");
    viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "plane_point_cloud");

    //-- Add normals
    viewer.addPointCloudNormals<pcl::PointXYZ, pcl::Normal> (source_cloud, cloud_normals, 1, 0.8, "normals");
    viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_COLOR, 0, 0, 255, "normals");

    //-- Add a plane
    //viewer.addPlane(*coefficients,"ransac_plane");
    //viewer.setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 0, 255, 0, "ransac_plane");

    //-- View contours
    pcl::visualization::PCLVisualizer viewer2("Contour");
    viewer2.addCoordinateSystem(1.0, "cloud", 0);
    viewer2.setBackgroundColor(0.05, 0.05, 0.05, 0);
    viewer2.addPointCloud(contour_points, cloud_color_handler, "contour_points");
    viewer2.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "contour_points");

    //-- Visualization thread
    while(!viewer.wasStopped() && !viewer2.wasStopped())
    {
        viewer.spinOnce();
        viewer2.spinOnce();
    }

    return 0;
}
