#ifndef ZBUFFERDEPTHIMAGECREATOR_H
#define ZBUFFERDEPTHIMAGECREATOR_H

#include <pcl/point_cloud.h>


template<typename PointT>
class ZBufferDepthImageCreator
{
    typedef typename pcl::PointCloud<PointT>::ConstPtr PointCloudConstPtr;

    public:
        ZBufferDepthImageCreator()
        {
            width = 0;
            height = 0;
        }

        void setInputPointCloud(const PointCloudConstPtr& pc)
        {
            point_cloud = pc;
        }

        bool compute()
        {
            return true;
        }

        bool setResolution(const int& width, const int& height)
        {
            if (width > 0 && height > 0)
            {
                this->width = width;
                this->height = height;
                return true;
            }
            else
            {
                return false;
            }
        }

        Eigen::MatrixXf getDepthImageAsMatrix()
        {
            return depth_image;
        }


    private:
        PointCloudConstPtr point_cloud;
        int width, height;
        Eigen::MatrixXf depth_image;
};



#endif // ZBUFFERDEPTHIMAGECREATOR_H
