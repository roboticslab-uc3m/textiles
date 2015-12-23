/*
 * Mesh Preprocessor
 *
 * Prepares the point cloud (obtained from the mesh) to be processed in later stages
 *
 */

#ifndef MESH_PREPROCESSOR_HPP
#define MESH_PREPROCESSOR_HPP

#include <pcl/point_cloud.h>
//-- Plane fitting
#include <pcl/ModelCoefficients.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
//-- Bounding box
#include <pcl/features/moment_of_inertia_estimation.h>
//-- Transform data
#include <pcl/filters/extract_indices.h>
#include <pcl/common/transforms.h>
//-- Passthrough filter
#include <pcl/filters/passthrough.h>
//-- Voxels
#include <pcl/filters/voxel_grid.h>
//-- Projection
#include <pcl/ModelCoefficients.h>
#include <pcl/filters/project_inliers.h>


template<typename PointT>
class MeshPreprocessor
{
    //-- Typedefs for clarity's sake
    typedef typename pcl::PointCloud<PointT>::ConstPtr PointCloudConstPtr;
    typedef typename pcl::PointCloud<PointT>::Ptr PointCloudPtr;
    typedef typename pcl::PointCloud<PointT> PointCloud;

    public:
        MeshPreprocessor() {
            //-- Set default values
            RANSAC_threshold_distance = 0.03;
        }

        void setRANSACThresholdDistance(float threshold_distance) {
            RANSAC_threshold_distance = threshold_distance;
        }

        void setInputCloud(const PointCloudConstPtr& input_cloud) {
            this->input_cloud = input_cloud;
        }

        bool process(pcl::PointCloud<PointT>& output_cloud) {
            //-- Downsampling the mesh prior to RANSAC
            PointCloudPtr downsampled_point_cloud(new PointCloud);
            typename pcl::VoxelGrid<PointT> voxel_grid_filter;
            voxel_grid_filter.setInputCloud(input_cloud);
            voxel_grid_filter.setLeafSize(0.01f, 0.01f, 0.01f); //-- 1cm^3
            voxel_grid_filter.filter(*downsampled_point_cloud);

            //-- Find table's plane
            //------------------------------------------------------------------------------------
            pcl::ModelCoefficients::Ptr table_plane_coefficients(new pcl::ModelCoefficients);
            pcl::PointIndices::Ptr table_plane_points(new pcl::PointIndices);
            typename pcl::SACSegmentation<PointT> segmentation;
            segmentation.setOptimizeCoefficients(true);
            segmentation.setModelType(pcl::SACMODEL_PLANE);
            segmentation.setMethodType(pcl::SAC_RANSAC);
            segmentation.setDistanceThreshold(RANSAC_threshold_distance);
            segmentation.setInputCloud(downsampled_point_cloud);
            segmentation.segment(*table_plane_points, *table_plane_coefficients);

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

            //-- Find points that do not belong to the plane
            //----------------------------------------------------------------------------------
            //-- Filter table points
            PointCloudPtr not_table_points (new PointCloud);
            typename pcl::ExtractIndices<PointT> extract_indices;
            extract_indices.setInputCloud(downsampled_point_cloud);
            extract_indices.setIndices(table_plane_points);
            extract_indices.setNegative(true);
            extract_indices.filter(*not_table_points);


            //-- Find bounding box:
            //-----------------------------------------------------------------------------------
            typename pcl::MomentOfInertiaEstimation<PointT> feature_extractor;
            PointT min_point_AABB, max_point_AABB;
            PointT min_point_OBB,  max_point_OBB;
            PointT position_OBB;
            Eigen::Matrix3f rotational_matrix_OBB;

            feature_extractor.setInputCloud(not_table_points);
            feature_extractor.compute();
            feature_extractor.getAABB(min_point_AABB, max_point_AABB);
            feature_extractor.getOBB(min_point_OBB, max_point_OBB, position_OBB, rotational_matrix_OBB);

            //-- Project transformation point onto table plane:
            typename pcl::ProjectInliers<PointT> project_inliners;
            PointCloudPtr OBB_position_to_be_projected(new PointCloud);
            OBB_position_to_be_projected->points.push_back(position_OBB);
            PointCloudPtr OBB_position_projected(new PointCloud);
            project_inliners.setModelType(pcl::SACMODEL_PLANE);
            project_inliners.setInputCloud(OBB_position_to_be_projected);
            project_inliners.setModelCoefficients(table_plane_coefficients);
            project_inliners.filter(*OBB_position_projected);
            PointT projected_OBB = OBB_position_projected->points[0];

            //-- Transform point cloud
            //-----------------------------------------------------------------------------------
            //-- Translating to center
            PointCloudPtr centered_cloud(new PointCloud);
            Eigen::Affine3f translation_transform = Eigen::Affine3f::Identity();
            translation_transform.translation() << -projected_OBB.x, -projected_OBB.y, -projected_OBB.z;
            pcl::transformPointCloud(*input_cloud, *centered_cloud, translation_transform);

            //-- Orient using the plane normal
            PointCloudPtr oriented_cloud(new PointCloud);
            Eigen::Vector3f normal_vector(table_plane_coefficients->values[0], table_plane_coefficients->values[1], table_plane_coefficients->values[2]);
            Eigen::Quaternionf rotation_quaternion = Eigen::Quaternionf().setFromTwoVectors(normal_vector, Eigen::Vector3f::UnitZ());
            pcl::transformPointCloud(*centered_cloud, *oriented_cloud, Eigen::Vector3f(0,0,0), rotation_quaternion);

            //-- Remove negative outliers (table noise)
            typename pcl::PassThrough<PointT> passthrough_filter;
            passthrough_filter.setInputCloud(oriented_cloud);
            passthrough_filter.setFilterFieldName("z");
            passthrough_filter.setFilterLimits(RANSAC_threshold_distance/2.0f, FLT_MAX);
            passthrough_filter.setFilterLimitsNegative(false);
            passthrough_filter.filter(output_cloud);

            return true;
        }


     private:
        float RANSAC_threshold_distance;

        PointCloudConstPtr input_cloud;
};

#endif // MESH_PREPROCESSOR_HPP
