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

        void setEnabled(bool enabled);
        void setAutoShow(bool enabled);

        template<typename PointT>
        bool plotPointCloud(typename pcl::PointCloud<PointT>::Ptr& point_cloud, const DebugColor& color)
        {
            return false;
        }

        bool show(std::string tag);

        //-- Available colors
        static const DebugColor RED;
        static const DebugColor BLUE;

    private:
        pcl::visualization::PCLVisualizer* current_viewer;
        bool enabled;
        bool auto_show;
};

const Debug::DebugColor Debug::RED = {255, 0, 0};
const Debug::DebugColor Debug::BLUE = {255, 0, 0};

void Debug::setEnabled(bool enabled)
{
    this->enabled = enabled;
}

void Debug::setAutoShow(bool enabled)
{


}

bool Debug::show(std::string tag="")
{
    return false;
}

#endif
