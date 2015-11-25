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
            width = 0;
            height = 0;
        }

        void setInputPointCloud(const PointCloudConstPtr& pc) { point_cloud = pc; }

        bool setResolution(const int& width, const int& height)
        {
            if (width > 0 && height > 0)
            {
                this->width = width;
                this->height = height;
                return true;
            }
            else
                return false;
        }

        Eigen::MatrixXf getDepthImageAsMatrix() { return depth_image; }


        bool compute()
        {
            if (width <= 0 || height <= 0)
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

            float bin_size_x = abs(max_point_AABB.x - min_point_AABB.x)/(float)width;
            float bin_size_y = abs(max_point_AABB.y - min_point_AABB.y)/(float)height;
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
        int width, height;
        Eigen::MatrixXf depth_image;
};



#endif // ZBUFFERDEPTHIMAGECREATOR_H
