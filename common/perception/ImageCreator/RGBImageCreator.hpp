#ifndef ZBUFFERDEPTHIMAGECREATOR_H
#define ZBUFFERDEPTHIMAGECREATOR_H

#include <pcl/point_cloud.h>
#include <pcl/filters/filter.h>
#include <pcl/features/moment_of_inertia_estimation.h>
#include <pcl/filters/extract_indices.h>

#include <cmath>

template<typename PointT>
class RGBImageCreator
{
    typedef typename pcl::PointCloud<PointT>::ConstPtr PointCloudConstPtr;

    public:
        RGBImageCreator() {
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

        Eigen::MatrixXf getChannelAsMatrix(int channel)
        {
            if (channel == CHANNEL_R)
                return r_image;
            else if (channel == CHANNEL_G)
                return g_image;
            else
                return b_image;
        }

        bool compute()
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
            std::cout << "Creating 2D image with resolution: " << width << "x" << height << "px" << std::endl;

            //-- Matrices to store image data
            this->r_image = Eigen::MatrixXf::Zero(height, width);
            this->g_image = Eigen::MatrixXf::Zero(height, width);
            this->b_image = Eigen::MatrixXf::Zero(height, width);
            Eigen::MatrixXf depth = Eigen::MatrixXf::Constant(height, width, lowest_height_limit);

            //-- Loop through those points to get RGBD data
            #pragma omp parallel for
            for (int i = 0; i < filtered_cloud->points.size(); i++)
            {
                if (isnan(filtered_cloud->points[i].x) || isnan(filtered_cloud->points[i].y ))
                    continue;

                int index_x = (filtered_cloud->points[i].x-min_point_bb.x) / average_point_distance;
                int index_y = (max_point_bb.y - filtered_cloud->points[i].y) / average_point_distance;

                if (index_x >= width) index_x = width-1;
                if (index_y >= height) index_y = height-1;

                //-- ZBuffer depth map output image
                float old_z;
                #pragma omp critical
                {
                    old_z = depth(index_y, index_x);
                    if (filtered_cloud->points[i].z > old_z)
                    {
                        depth(index_y, index_x) = filtered_cloud->points[i].z;
                        this->r_image(index_y, index_x) = filtered_cloud->points[i].r;
                        this->g_image(index_y, index_x) = filtered_cloud->points[i].g;
                        this->b_image(index_y, index_x) = filtered_cloud->points[i].b;
                    }
                }
            }
            return true;
        }

    static const int CHANNEL_R = 0;
    static const int CHANNEL_G = 1;
    static const int CHANNEL_B = 2;

    private:
        PointCloudConstPtr point_cloud;
        float average_point_distance;
        //-- Bounding Box
        bool user_defined_bb;
        PointT min_point_bb, max_point_bb;
        //-- Output images
        Eigen::MatrixXf r_image, g_image, b_image;
};



#endif // ZBUFFERDEPTHIMAGECREATOR_H
