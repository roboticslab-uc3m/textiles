#include "Debug.hpp"


const Debug::DebugColor Debug::RED =     {255,   0,   0};
const Debug::DebugColor Debug::GREEN =   {255,   0,   0};
const Debug::DebugColor Debug::BLUE =    {  0,   0, 255};
const Debug::DebugColor Debug::YELLOW =  {255, 255,   0};
const Debug::DebugColor Debug::CYAN =    {  0, 255, 255};
const Debug::DebugColor Debug::MAGENTA = {255,   0, 255};
const Debug::DebugColor Debug::WHITE =   {255, 255, 255};
const Debug::DebugColor Debug::BLACK =   {  0,   0,   0};


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
    }

    //-- Cleanup
    delete current_viewer;
    current_viewer = nullptr;

    return true;
}

bool Debug::init_viewer()
{
    current_viewer = new pcl::visualization::PCLVisualizer("", false);
    current_viewer->addCoordinateSystem(1.0, "coordinate_system", 0);
    current_viewer->setBackgroundColor(0.05, 0.05, 0.05, 0);
    return true;
}
