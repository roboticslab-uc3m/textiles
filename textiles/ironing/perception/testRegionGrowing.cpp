#include <iostream>
#include <vector>
#include <pcl/point_types.h>
#include <pcl/console/parse.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/search/search.h>
#include <pcl/search/kdtree.h>
#include <pcl/features/normal_3d.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/filters/passthrough.h>
#include <pcl/segmentation/region_growing.h>
#include <pcl/features/moment_of_inertia_estimation.h>

#include "Debug.hpp"

int main (int argc, char** argv)
{
    Debug debug;
    debug.setAutoShow(false);
    debug.setEnabled(false);

    //-- Get point cloud file from arguments
    std::vector<int> filenames;
    bool file_is_pcd = false;

    filenames = pcl::console::parse_file_extension_argument(argc, argv, ".ply");

    if (filenames.size() != 1)
    {
        filenames = pcl::console::parse_file_extension_argument(argc, argv, ".pcd");

        if (filenames.size() != 1)
            return -1;

        file_is_pcd = true;
    }

    //-- Load point cloud data
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr source_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);

    if (file_is_pcd)
    {
        if (pcl::io::loadPCDFile(argv[filenames[0]], *source_cloud) < 0)
        {
            std::cout << "Error loading colored point cloud " << argv[filenames[0]] << std::endl << std::endl;
            return -1;
        }
    }
    else
    {
        if (pcl::io::loadPLYFile(argv[filenames[0]], *source_cloud) < 0)
        {
            std::cout << "Error loading colored point cloud " << argv[filenames[0]] << std::endl << std::endl;
            return -1;
        }
    }

  pcl::search::Search<pcl::PointXYZRGB>::Ptr tree = boost::shared_ptr<pcl::search::Search<pcl::PointXYZRGB> > (new pcl::search::KdTree<pcl::PointXYZRGB>);
  pcl::PointCloud <pcl::Normal>::Ptr normals (new pcl::PointCloud <pcl::Normal>);
  pcl::NormalEstimation<pcl::PointXYZRGB, pcl::Normal> normal_estimator;
  normal_estimator.setSearchMethod (tree);
  normal_estimator.setInputCloud (source_cloud);
  normal_estimator.setKSearch (50);
  normal_estimator.compute (*normals);

  pcl::RegionGrowing<pcl::PointXYZRGB, pcl::Normal> reg;
  reg.setMinClusterSize(150);
  reg.setMaxClusterSize(1000000);
  reg.setSearchMethod(tree);
  reg.setNumberOfNeighbours(35);
  reg.setInputCloud(source_cloud);
  reg.setInputNormals(normals);
  reg.setSmoothnessThreshold(3.0 / 180.0 * M_PI);
  reg.setCurvatureThreshold(1.0);

  std::vector <pcl::PointIndices> clusters;
  reg.extract (clusters);
  pcl::PointCloud <pcl::PointXYZRGB>::Ptr colored_cloud = reg.getColoredCloud();
  debug.setEnabled(false);
  debug.plotPointCloud<pcl::PointXYZRGB>(colored_cloud, Debug::COLOR_RED);
  debug.show("Clusters");

//  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZRGB>);


  //-----------------------------------------------------------------------------------
  //-- Bounding box filter:
  //-----------------------------------------------------------------------------------
  pcl::MomentOfInertiaEstimation<pcl::PointXYZRGB> feature_extractor;
  pcl::PointXYZRGB min_point_AABB, max_point_AABB;
  pcl::PointXYZRGB min_point_OBB,  max_point_OBB;
  pcl::PointXYZRGB position_OBB;
  Eigen::Matrix3f rotational_matrix_OBB;

  feature_extractor.setInputCloud(colored_cloud);
  feature_extractor.compute();
  feature_extractor.getAABB(min_point_AABB, max_point_AABB);
  feature_extractor.getOBB(min_point_OBB, max_point_OBB, position_OBB, rotational_matrix_OBB);

  debug.setEnabled(true);
  debug.plotPointCloud<pcl::PointXYZRGB>(colored_cloud, Debug::COLOR_ORIGINAL);

  //debug.getRawViewer()->addCube (min_point_AABB.x, max_point_AABB.x, min_point_AABB.y, max_point_AABB.y, min_point_AABB.z, max_point_AABB.z, 1.0, 1.0, 0.0, "AABB");

  float factor_x = 0.8;
  float factor_y = 0.8;
  float factor_z = 1.0;

  Eigen::Vector3f p1 (min_point_OBB.x*factor_x, min_point_OBB.y*factor_y, min_point_OBB.z*factor_z);
  Eigen::Vector3f p2 (min_point_OBB.x*factor_x, min_point_OBB.y*factor_y, max_point_OBB.z*factor_z);
  Eigen::Vector3f p3 (max_point_OBB.x*factor_x, min_point_OBB.y*factor_y, max_point_OBB.z*factor_z);
  Eigen::Vector3f p4 (max_point_OBB.x*factor_x, min_point_OBB.y*factor_y, min_point_OBB.z*factor_z);
  Eigen::Vector3f p5 (min_point_OBB.x*factor_x, max_point_OBB.y*factor_y, min_point_OBB.z*factor_z);
  Eigen::Vector3f p6 (min_point_OBB.x*factor_x, max_point_OBB.y*factor_y, max_point_OBB.z*factor_z);
  Eigen::Vector3f p7 (max_point_OBB.x*factor_x, max_point_OBB.y*factor_y, max_point_OBB.z*factor_z);
  Eigen::Vector3f p8 (max_point_OBB.x*factor_x, max_point_OBB.y*factor_y, min_point_OBB.z*factor_z);

  Eigen::Vector3f position (position_OBB.x, position_OBB.y, position_OBB.z);
  Eigen::Quaternionf quat (rotational_matrix_OBB);
  //debug.getRawViewer()->addCube (position, quat, max_point_OBB.x - min_point_OBB.x, max_point_OBB.y - min_point_OBB.y, max_point_OBB.z - min_point_OBB.z, "OBB");

  p1 = rotational_matrix_OBB * p1 + position;
  p2 = rotational_matrix_OBB * p2 + position;
  p3 = rotational_matrix_OBB * p3 + position;
  p4 = rotational_matrix_OBB * p4 + position;
  p5 = rotational_matrix_OBB * p5 + position;
  p6 = rotational_matrix_OBB * p6 + position;
  p7 = rotational_matrix_OBB * p7 + position;
  p8 = rotational_matrix_OBB * p8 + position;

  pcl::PointXYZ pt1 (p1 (0), p1 (1), p1 (2));
  pcl::PointXYZ pt2 (p2 (0), p2 (1), p2 (2));
  pcl::PointXYZ pt3 (p3 (0), p3 (1), p3 (2));
  pcl::PointXYZ pt4 (p4 (0), p4 (1), p4 (2));
  pcl::PointXYZ pt5 (p5 (0), p5 (1), p5 (2));
  pcl::PointXYZ pt6 (p6 (0), p6 (1), p6 (2));
  pcl::PointXYZ pt7 (p7 (0), p7 (1), p7 (2));
  pcl::PointXYZ pt8 (p8 (0), p8 (1), p8 (2));

  debug.getRawViewer()->addLine (pt1, pt2, 1.0, 0.0, 0.0, "1 edge");
  debug.getRawViewer()->addLine (pt1, pt4, 1.0, 0.0, 0.0, "2 edge");
  debug.getRawViewer()->addLine (pt1, pt5, 1.0, 0.0, 0.0, "3 edge");
  debug.getRawViewer()->addLine (pt5, pt6, 1.0, 0.0, 0.0, "4 edge");
  debug.getRawViewer()->addLine (pt5, pt8, 1.0, 0.0, 0.0, "5 edge");
  debug.getRawViewer()->addLine (pt2, pt6, 1.0, 0.0, 0.0, "6 edge");
  debug.getRawViewer()->addLine (pt6, pt7, 1.0, 0.0, 0.0, "7 edge");
  debug.getRawViewer()->addLine (pt7, pt8, 1.0, 0.0, 0.0, "8 edge");
  debug.getRawViewer()->addLine (pt2, pt3, 1.0, 0.0, 0.0, "9 edge");
  debug.getRawViewer()->addLine (pt4, pt8, 1.0, 0.0, 0.0, "10 edge");
  debug.getRawViewer()->addLine (pt3, pt4, 1.0, 0.0, 0.0, "11 edge");
  debug.getRawViewer()->addLine (pt3, pt7, 1.0, 0.0, 0.0, "12 edge");


  pcl::PointXYZ origin(0,0,0);
  debug.getRawViewer()->addLine(origin, position_OBB, 1.0, 1.0, 0.0, "positionobb");

  debug.show();

  return (0);
}
