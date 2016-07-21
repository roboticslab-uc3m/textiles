#ifndef __DEBUG_HPP__
#define __DEBUG_HPP__

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/visualization/pcl_visualizer.h>


class Debug
{
    public:
        struct DebugColor {
            int r;
            int g;
            int b;
        };

        typedef struct DebugColor DebugColor;

        Debug();
        ~Debug();

        void setEnabled(bool enabled);
        void setAutoShow(bool enabled);

        template<typename PointT>
        bool plotPointCloud(typename pcl::PointCloud<PointT>::Ptr& point_cloud,
                            const DebugColor& color, int point_size = 1);

        template<typename PointT, typename PointNT>
        bool plotNormals(typename pcl::PointCloud<PointT>::Ptr& cloud,
                         typename pcl::PointCloud<PointNT>::Ptr& normals,
                         const DebugColor& color, int density = 100, float scale = 0.02);

        bool plotPlane(pcl::ModelCoefficients plane_coefficients, const DebugColor& color);
        bool plotPlane(double A, double B, double C, double D, const DebugColor& color);

        template<typename PointT>
        bool plotBoundingBox(PointT min_point, PointT max_point, PointT position,
                             Eigen::Matrix3f rotational_matrix, const DebugColor& color, bool solid = false);

        bool show(std::string tag = "");

        pcl::visualization::PCLVisualizer* getRawViewer();

        //-- Available colors
        static const DebugColor COLOR_RED;
        static const DebugColor COLOR_GREEN;
        static const DebugColor COLOR_BLUE;
        static const DebugColor COLOR_YELLOW;
        static const DebugColor COLOR_CYAN;
        static const DebugColor COLOR_MAGENTA;
        static const DebugColor COLOR_WHITE;
        static const DebugColor COLOR_BLACK;
        static const DebugColor COLOR_ORIGINAL; //-- Do not use with non-colored point clouds

    private:
        bool init_viewer();

        pcl::visualization::PCLVisualizer* current_viewer;
        bool enabled;
        bool auto_show;
        int uuid_counter;
};

template<typename PointT, typename PointNT>
bool Debug::plotNormals(typename pcl::PointCloud<PointT>::Ptr& cloud,
                        typename pcl::PointCloud<PointNT>::Ptr &normals,
                        const Debug::DebugColor &color, int density, float scale)
{
    if (current_viewer == nullptr)
        if (!init_viewer())
            return false;

    //-- Craft uuid string for current uuid
    std::string uuid_str = std::to_string(uuid_counter);
    uuid_counter++;

    //-- Add normals to the viewer
    current_viewer->addPointCloudNormals<PointT, PointNT>(cloud,normals, density,
                                                          scale, uuid_str);

    if (auto_show)
        return show("auto");

    return true;
}

template<typename PointT>
bool Debug::plotPointCloud(typename pcl::PointCloud<PointT>::Ptr& point_cloud,
                    const Debug::DebugColor& color, int point_size)
{
    if (current_viewer == nullptr)
        if (!init_viewer())
            return false;

    //-- Craft uuid string for current uuid
    std::string uuid_str = std::to_string(uuid_counter);
    uuid_counter++;

    //-- Create color handler
    pcl::visualization::PointCloudColorHandlerCustom<PointT> color_handler(point_cloud,
                                                                           color.r, color.g, color.b);
    //-- Add cloud to viewer
    current_viewer->addPointCloud(point_cloud, color_handler, uuid_str);
    current_viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE,
                                                     point_size, uuid_str);

    if (auto_show)
        return show("auto");

    return true;
}

template<typename PointT>
bool Debug::plotBoundingBox(PointT min_point, PointT max_point, PointT center,
                            Eigen::Matrix3f rotational_matrix, const Debug::DebugColor &color, bool solid)
{
    if (current_viewer == nullptr)
        if (!init_viewer())
            return false;

    //-- Craft uuid string for current uuid
    std::string uuid_str = std::to_string(uuid_counter);
    uuid_counter++;

    //-- Create points to calculate lines
    Eigen::Vector3f position (center.x, center.y, center.z);
    Eigen::Vector3f p1 (min_point.x, min_point.y, min_point.z);
    Eigen::Vector3f p2 (min_point.x, min_point.y, max_point.z);
    Eigen::Vector3f p3 (max_point.x, min_point.y, max_point.z);
    Eigen::Vector3f p4 (max_point.x, min_point.y, min_point.z);
    Eigen::Vector3f p5 (min_point.x, max_point.y, min_point.z);
    Eigen::Vector3f p6 (min_point.x, max_point.y, max_point.z);
    Eigen::Vector3f p7 (max_point.x, max_point.y, max_point.z);
    Eigen::Vector3f p8 (max_point.x, max_point.y, min_point.z);

    p1 = rotational_matrix * p1 + position;
    p2 = rotational_matrix * p2 + position;
    p3 = rotational_matrix * p3 + position;
    p4 = rotational_matrix * p4 + position;
    p5 = rotational_matrix * p5 + position;
    p6 = rotational_matrix * p6 + position;
    p7 = rotational_matrix * p7 + position;
    p8 = rotational_matrix * p8 + position;

    //-- Go back to pcl data types
    pcl::PointXYZ pt1 (p1 (0), p1 (1), p1 (2));
    pcl::PointXYZ pt2 (p2 (0), p2 (1), p2 (2));
    pcl::PointXYZ pt3 (p3 (0), p3 (1), p3 (2));
    pcl::PointXYZ pt4 (p4 (0), p4 (1), p4 (2));
    pcl::PointXYZ pt5 (p5 (0), p5 (1), p5 (2));
    pcl::PointXYZ pt6 (p6 (0), p6 (1), p6 (2));
    pcl::PointXYZ pt7 (p7 (0), p7 (1), p7 (2));
    pcl::PointXYZ pt8 (p8 (0), p8 (1), p8 (2));

    //-- Draw stuff
    current_viewer->addLine (pt1, pt2, 1.0, 0.0, 0.0, "1 edge "+uuid_str);
    current_viewer->addLine (pt1, pt4, 1.0, 0.0, 0.0, "2 edge "+uuid_str);
    current_viewer->addLine (pt1, pt5, 1.0, 0.0, 0.0, "3 edge "+uuid_str);
    current_viewer->addLine (pt5, pt6, 1.0, 0.0, 0.0, "4 edge "+uuid_str);
    current_viewer->addLine (pt5, pt8, 1.0, 0.0, 0.0, "5 edge "+uuid_str);
    current_viewer->addLine (pt2, pt6, 1.0, 0.0, 0.0, "6 edge "+uuid_str);
    current_viewer->addLine (pt6, pt7, 1.0, 0.0, 0.0, "7 edge "+uuid_str);
    current_viewer->addLine (pt7, pt8, 1.0, 0.0, 0.0, "8 edge "+uuid_str);
    current_viewer->addLine (pt2, pt3, 1.0, 0.0, 0.0, "9 edge "+uuid_str);
    current_viewer->addLine (pt4, pt8, 1.0, 0.0, 0.0, "10 edge "+uuid_str);
    current_viewer->addLine (pt3, pt4, 1.0, 0.0, 0.0, "11 edge "+uuid_str);
    current_viewer->addLine (pt3, pt7, 1.0, 0.0, 0.0, "12 edge "+uuid_str);

    if (auto_show)
        return show("auto");

    return true;
}

#endif
