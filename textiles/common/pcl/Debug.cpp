#include "Debug.hpp"

const Debug::DebugColor Debug::COLOR_RED =     {255,   0,   0};
const Debug::DebugColor Debug::COLOR_GREEN =   {  0, 255,   0};
const Debug::DebugColor Debug::COLOR_BLUE =    {  0,   0, 255};
const Debug::DebugColor Debug::COLOR_YELLOW =  {255, 255,   0};
const Debug::DebugColor Debug::COLOR_CYAN =    {  0, 255, 255};
const Debug::DebugColor Debug::COLOR_MAGENTA = {255,   0, 255};
const Debug::DebugColor Debug::COLOR_WHITE =   {255, 255, 255};
const Debug::DebugColor Debug::COLOR_BLACK =   {  0,   0,   0};
const Debug::DebugColor Debug::COLOR_ORIGINAL ={ -1,  -1,  -1};

Debug::Debug()
{
    enabled = false;
    auto_show = false;
    uuid_counter = 0;
    current_viewer = nullptr;
}

Debug::~Debug()
{
    if (current_viewer != nullptr)
    {
        delete current_viewer;
        current_viewer = nullptr;
    }
}

void Debug::setEnabled(bool enabled)
{
    this->enabled = enabled;
}

void Debug::setAutoShow(bool enabled)
{
    auto_show = enabled;
}

bool Debug::plotPlane(pcl::ModelCoefficients plane_coefficients, const Debug::DebugColor &color)
{
    if (current_viewer == nullptr)
        if (!init_viewer())
            return false;

    //-- Craft uuid string for current uuid
    std::string uuid_str = std::to_string(uuid_counter);
    uuid_counter++;

    current_viewer->addPlane(plane_coefficients, uuid_str);
    current_viewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR,
                                                color.r/255., color.g/255., color.b/255.,
                                                uuid_str);

    if (auto_show)
        return show("auto");

    return true;
}

bool Debug::plotPlane(double A, double B, double C, double D, const Debug::DebugColor &color)
{
    //-- Pack values
    pcl::ModelCoefficients coeffs;
    coeffs.values.push_back(A);
    coeffs.values.push_back(B);
    coeffs.values.push_back(C);
    coeffs.values.push_back(D);

    //-- Plot
    this->plotPlane(coeffs, color);
}

bool Debug::show(std::string tag)
{
    if (current_viewer == nullptr)
        return false;

    if (enabled)
    {
        if (tag != "")
            current_viewer->setWindowName(tag);

        //-- Visualization thread
        current_viewer->createInteractor();
        while(!current_viewer->wasStopped())
            current_viewer->spinOnce();

        current_viewer->close();
    }

    //-- Cleanup
    delete current_viewer;
    current_viewer = nullptr;

    return true;
}

pcl::visualization::PCLVisualizer *Debug::getRawViewer()
{
    return current_viewer;
}

bool Debug::init_viewer()
{
    current_viewer = new pcl::visualization::PCLVisualizer("", false);
    current_viewer->addCoordinateSystem(1.0, "coordinate_system", 0);
    current_viewer->setBackgroundColor(0.05, 0.05, 0.05, 0);
    return true;
}

template<>
bool Debug::plotPointCloud<pcl::PointXYZRGB>(typename pcl::PointCloud<pcl::PointXYZRGB>::Ptr& point_cloud,
                    const DebugColor& color, int point_size)
{
    if (current_viewer == nullptr)
        if (!init_viewer())
            return false;

    //-- Craft uuid string for current uuid
    std::string uuid_str = std::to_string(uuid_counter);
    uuid_counter++;


    //-- Create color handler
    pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> rgb_handler(point_cloud);

    //-- Add cloud to viewer
    current_viewer->addPointCloud(point_cloud, rgb_handler, uuid_str);
    current_viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE,
                                                     point_size, uuid_str);

    if (auto_show)
        return show("auto");

    return true;
}

