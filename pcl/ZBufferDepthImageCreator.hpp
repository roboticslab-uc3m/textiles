#ifndef ZBUFFERDEPTHIMAGECREATOR_H
#define ZBUFFERDEPTHIMAGECREATOR_H

#include <pcl/point_cloud.h>
#include <pcl/features/moment_of_inertia_estimation.h>

template<typename PointT>
class ZBufferDepthImageCreator
{
    typedef typename pcl::PointCloud<PointT>::ConstPtr PointCloudConstPtr;

    public:
        ZBufferDepthImageCreator() {
            resolution = 0;
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

        Eigen::MatrixXf getDepthImageAsMatrix() { return depth_image; }


        bool compute()
        {
            if (resolution <= 0)
            {
                std::cerr << "Error: resolution not set" << std::endl;
                return false;
            }

            //-- Find bounding box of input point_cloud
            pcl::MomentOfInertiaEstimation<PointT> feature_extractor;
            PointT min_point_AABB, max_point_AABB;
            PointT min_point_OBB,  max_point_OBB;
            PointT position_OBB;
            Eigen::Matrix3f rotational_matrix_OBB;

            feature_extractor.setInputCloud(point_cloud);
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

            for (int i = 0; i < point_cloud->points.size(); i++)
            {
                int index_x = (point_cloud->points[i].x-min_point_AABB.x) / bin_size_x;
                int index_y = (max_point_AABB.y - point_cloud->points[i].y) / bin_size_y;

                if (index_x >= width) index_x = width-1;
                if (index_y >= height) index_y = height-1;

                //std::cout << "Point " << i << " in bin (" << index_x << ", " << index_y << ")" << std::endl;

                float old_z = depth_image(index_y, index_x);
                if (point_cloud->points[i].z > old_z)
                    depth_image(index_y, index_x) = point_cloud->points[i].z;
            }
            return true;
        }


    private:
        PointCloudConstPtr point_cloud;
        int resolution;
        Eigen::MatrixXf depth_image;
};



#endif // ZBUFFERDEPTHIMAGECREATOR_H
