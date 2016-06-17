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
                            const DebugColor& color, int point_size = 1)
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

        bool plotPlane(pcl::ModelCoefficients plane_coefficients, const DebugColor& color);
        bool plotPlane(double A, double B, double C, double D, const DebugColor& color);

        bool show(std::string tag = "");

        //-- Available colors
        static const DebugColor COLOR_RED;
        static const DebugColor COLOR_GREEN;
        static const DebugColor COLOR_BLUE;
        static const DebugColor COLOR_YELLOW;
        static const DebugColor COLOR_CYAN;
        static const DebugColor COLOR_MAGENTA;
        static const DebugColor COLOR_WHITE;
        static const DebugColor COLOR_BLACK;

    private:
        bool init_viewer();

        pcl::visualization::PCLVisualizer* current_viewer;
        bool enabled;
        bool auto_show;
        int uuid_counter;
};

#endif
