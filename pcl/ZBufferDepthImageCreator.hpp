#ifndef ZBUFFERDEPTHIMAGECREATOR_H
#define ZBUFFERDEPTHIMAGECREATOR_H

#include <pcl/point_cloud.h>
#include <pcl/filters/filter.h>
#include <pcl/features/moment_of_inertia_estimation.h>
#include <pcl/surface/mls.h> //-- Upsampling

template<typename PointT>
class ZBufferDepthImageCreator
{
    typedef typename pcl::PointCloud<PointT>::ConstPtr PointCloudConstPtr;

    public:
        ZBufferDepthImageCreator() {
            resolution = 0;
            do_upsampling = false;
        }

        void setInputPointCloud(const PointCloudConstPtr& pc) { point_cloud = pc; }

        bool setResolution(const int& resolution)
        {
            if (resolution > 0)
            {
                this->resolution = resolution;
                return true;
            }
            else
                return false;
        }

        bool setUpsampling(bool do_upsampling) { this->do_upsampling = do_upsampling; }

        Eigen::MatrixXf getDepthImageAsMatrix() { return depth_image; }


        bool compute()
        {
            if (resolution <= 0)
            {
                std::cerr << "Error: resolution not set" << std::endl;
                return false;
            }

            //-- Upsampling (if enabled)
            typename pcl::PointCloud<PointT>::Ptr processed_cloud(new pcl::PointCloud<PointT>);
            if (do_upsampling)
            {
                pcl::MovingLeastSquares<PointT, PointT> mls_filter;
                typename pcl::search::KdTree<PointT>::Ptr kd_tree;
                mls_filter.setInputCloud(point_cloud);
                mls_filter.setSearchMethod(kd_tree);
                mls_filter.setSearchRadius(3);
                mls_filter.setUpsamplingMethod(pcl::MovingLeastSquares<PointT, PointT>::SAMPLE_LOCAL_PLANE);
                mls_filter.setUpsamplingRadius(3);
                mls_filter.setUpsamplingStepSize(1);
                mls_filter.process(*processed_cloud);

                std::cout << "Upsampling from " << point_cloud->points.size()
                          << "points to " << processed_cloud->points.size()
                          << " points." << std::endl;

                std::vector<int> mapping;
                pcl::removeNaNFromPointCloud(*processed_cloud, *processed_cloud, mapping);

                std::cout << "After NaN removal: " << processed_cloud->points.size() << " points." << std::endl;

                if (processed_cloud->points.size() == 0)
                {
                    std::cerr << "Some error happened at upsampling state. Aborting..." << std::endl;
                    return false;
                }
            }
            else
            {
                *processed_cloud = *point_cloud;
            }

            //-- Find bounding box of input point_cloud
            pcl::MomentOfInertiaEstimation<PointT> feature_extractor;
            PointT min_point_AABB, max_point_AABB;
            PointT min_point_OBB,  max_point_OBB;
            PointT position_OBB;
            Eigen::Matrix3f rotational_matrix_OBB;

            feature_extractor.setInputCloud(processed_cloud);
            feature_extractor.compute();
            feature_extractor.getAABB(min_point_AABB, max_point_AABB);
            feature_extractor.getOBB(min_point_OBB, max_point_OBB, position_OBB, rotational_matrix_OBB);

            //-- Calculate aspect ratio and bin size
            int AABB_width = abs(max_point_AABB.x - min_point_AABB.x);
            int AABB_height = abs(max_point_AABB.y - min_point_AABB.y);
            float aspect_ratio = AABB_width / (float)AABB_height;

            int width, height;
            if (aspect_ratio >= 1)
            {
                width = resolution;
                height = resolution/aspect_ratio;
            }
            else
            {
                height = resolution;
                width = resolution*aspect_ratio;
            }

            float bin_size_x = AABB_width/(float)width;
            float bin_size_y = AABB_height/(float)height;

            //-- Fill bins with z values

            this->depth_image = Eigen::MatrixXf::Zero(height, width);

            for (int i = 0; i < processed_cloud->points.size(); i++)
            {
                if (isnan(processed_cloud->points[i].x) || isnan(processed_cloud->points[i].y ))
                    continue;

                int index_x = (processed_cloud->points[i].x-min_point_AABB.x) / bin_size_x;
                int index_y = (max_point_AABB.y - processed_cloud->points[i].y) / bin_size_y;

                if (index_x >= width) index_x = width-1;
                if (index_y >= height) index_y = height-1;

                std::cout << "Point " << i << "(" << processed_cloud->points[i].x << ", "
                          << processed_cloud->points[i].y << ", "<< processed_cloud->points[i].z
                          << ") in bin (" << index_x << ", " << index_y << ")" << std::endl;

                float old_z = depth_image(index_y, index_x);
                if (processed_cloud->points[i].z > old_z)
                    depth_image(index_y, index_x) = processed_cloud->points[i].z;
            }
            return true;
        }


    private:
        PointCloudConstPtr point_cloud;
        int resolution;
        bool do_upsampling;
        Eigen::MatrixXf depth_image;
};



#endif // ZBUFFERDEPTHIMAGECREATOR_H
