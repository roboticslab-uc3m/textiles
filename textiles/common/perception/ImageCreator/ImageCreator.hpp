#ifndef __ImageCreator_HPP__
#define __ImageCreator_HPP__

#include <pcl/point_cloud.h>
#include <pcl/filters/filter.h>
#include <pcl/features/moment_of_inertia_estimation.h>
#include <pcl/filters/extract_indices.h>

#include <cmath>

template<typename PointT>
class ImageCreator
{
    typedef typename pcl::PointCloud<PointT>::ConstPtr PointCloudConstPtr;

    public:
        ImageCreator() {
            user_defined_bb = false;
        }

        void setInputPointCloud(const PointCloudConstPtr& pc) { point_cloud = pc; }
        void setAvgPointDist(const float& average_point_distance) { this->average_point_distance = average_point_distance; }
        void setBoundingBox(PointT min_point_bb, PointT max_point_bb)
        {
            this->min_point_bb = min_point_bb;
            this->max_point_bb = max_point_bb;
            this->user_defined_bb = true;
        }

        virtual Eigen::MatrixXf getImageAsMatrix(const int& type) = 0;

        virtual bool compute() = 0;

    protected:
        bool filterPointcloud()
        {
            if (average_point_distance <= 0)
            {
                std::cerr << "Error: average point distance not set" << std::endl;
                return false;
            }

            float lowest_height_limit = 0;
            typename pcl::PointCloud<PointT>::Ptr filtered_cloud(new pcl::PointCloud<PointT>);
            if (!user_defined_bb)
            {
                *filtered_cloud = *point_cloud;

                //-- Find bounding box of input point_cloud
                pcl::MomentOfInertiaEstimation<PointT> feature_extractor;
                feature_extractor.setInputCloud(filtered_cloud);
                feature_extractor.compute();
                feature_extractor.getAABB(min_point_bb, max_point_bb);
                lowest_height_limit = min_point_bb.z;
            }
            else
            {
                //-- User defined bounding box to use: filter the cloud with the bounding box
                lowest_height_limit = min_point_bb.z;
                pcl::octree::OctreePointCloudSearch<PointT> octree(average_point_distance/2.0f);
                std::vector<int> points_within_bounding_box;
                Eigen::Vector3f min_bb(min_point_bb.x, min_point_bb.y, lowest_height_limit);
                Eigen::Vector3f max_bb(max_point_bb.x, max_point_bb.y, 1);
                octree.setInputCloud(point_cloud->makeShared());
                octree.addPointsFromInputCloud();
                octree.boxSearch(min_bb, max_bb, points_within_bounding_box);

                pcl::ExtractIndices<PointT> extract_indices;
                pcl::PointIndices::Ptr indices(new pcl::PointIndices);
                indices->indices= points_within_bounding_box;
                extract_indices.setInputCloud(point_cloud);
                extract_indices.setIndices(indices);
                extract_indices.setNegative(false);
                extract_indices.filter(*filtered_cloud);
            }

            //-- Calculate image resolution
            /* Note: if not using std::abs, floating abs function seems to be
             * not supported :-/ */
            float bb_width = std::abs(max_point_bb.x - min_point_bb.x);
            float bb_height = std::abs(max_point_bb.y - min_point_bb.y);

            int width = std::ceil(bb_width / average_point_distance);
            int height = std::ceil(bb_height / average_point_distance);


            return true;
        }

    private:
        PointCloudConstPtr point_cloud;
        float average_point_distance;
        //-- Bounding Box
        bool user_defined_bb;
        PointT min_point_bb, max_point_bb;
};



#endif // __DepthImageCreator_HPP__
